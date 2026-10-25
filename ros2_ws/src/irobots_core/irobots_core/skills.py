"""Skill library and registry (PLAN:irobots-v2:C10).

Enforces Invariant I1: the LLM never emits a Cartesian coordinate. Every skill
takes at most a tracked object/target ID, and the host derives the Cartesian
target pose from perception at execution time.

Also enforces Invariant I4 at the spec level: every skill carries an explicit
per-skill deadline.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Set
from dataclasses import dataclass, field
from typing import Any, Final


class SkillValidationError(ValueError):
    """Raised when a skill call violates the registry schema or references an unknown target."""


@dataclass(frozen=True, slots=True)
class SkillCall:
    """One atomic skill invocation emitted by the worker or scripted policy (I1, I3)."""

    skill: str
    target_id: str | None = None
    arguments: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class SkillSpec:
    """Specification of one atomic robot skill."""

    name: str
    description: str
    requires_target: bool
    parameters: Mapping[str, str]
    expected_outcome: str
    deadline_s: float
    precondition: Callable[[SkillCall, Set[str]], tuple[bool, str]]
    verification_predicate: Callable[[Mapping[str, Any], Mapping[str, Any]], bool]


def _precond_requires_target(call: SkillCall, known_ids: Set[str]) -> tuple[bool, str]:
    if not call.target_id:
        return (False, f"skill {call.skill!r} requires a target_id")
    if call.target_id not in known_ids:
        return (False, f"unknown_target: {call.target_id!r} not in tracked scene ids")
    return (True, "ok")


def _precond_no_target(call: SkillCall, _known_ids: Set[str]) -> tuple[bool, str]:
    if call.target_id is not None and call.target_id != "":
        return (False, f"skill {call.skill!r} does not take a target_id")
    return (True, "ok")


def _pred_gripper_at_pose(_before: Mapping[str, Any], after: Mapping[str, Any]) -> bool:
    return bool(after.get("gripper_at_pose", False))


def _pred_object_grasped(_before: Mapping[str, Any], after: Mapping[str, Any]) -> bool:
    return bool(after.get("object_grasped", False) or after.get("gripper_at_pose", False))


def _pred_object_lifted(_before: Mapping[str, Any], after: Mapping[str, Any]) -> bool:
    return bool(after.get("object_lifted", False))


def _pred_object_released(_before: Mapping[str, Any], after: Mapping[str, Any]) -> bool:
    return bool(after.get("object_released", False) or after.get("gripper_at_pose", False))


SKILL_REGISTRY: Final[Mapping[str, SkillSpec]] = {
    "move_above": SkillSpec(
        name="move_above",
        description="Move the open gripper to a safe hover height directly above a tracked target.",
        requires_target=True,
        parameters={"target_id": "Persistent tracked id of the object or bin"},
        expected_outcome="gripper_at_pose",
        deadline_s=5.0,
        precondition=_precond_requires_target,
        verification_predicate=_pred_gripper_at_pose,
    ),
    "grasp": SkillSpec(
        name="grasp",
        description="Descend to the tracked object's grasp height aligned with its principal axis and close jaws.",
        requires_target=True,
        parameters={"target_id": "Persistent tracked id of the object to grasp"},
        expected_outcome="object_moved_with_gripper",
        deadline_s=6.0,
        precondition=_precond_requires_target,
        verification_predicate=_pred_object_grasped,
    ),
    "lift": SkillSpec(
        name="lift",
        description="Lift the currently grasped tracked object to transport height.",
        requires_target=True,
        parameters={"target_id": "Persistent tracked id of the grasped object"},
        expected_outcome="object_lifted",
        deadline_s=5.0,
        precondition=_precond_requires_target,
        verification_predicate=_pred_object_lifted,
    ),
    "release": SkillSpec(
        name="release",
        description="Open the gripper jaws to release the held object above the current target.",
        requires_target=False,
        parameters={},
        expected_outcome="gripper_at_pose",
        deadline_s=3.0,
        precondition=_precond_no_target,
        verification_predicate=_pred_object_released,
    ),
    "retreat": SkillSpec(
        name="retreat",
        description="Raise the gripper vertically clear of the workspace after releasing or aborting.",
        requires_target=False,
        parameters={},
        expected_outcome="gripper_at_pose",
        deadline_s=4.0,
        precondition=_precond_no_target,
        verification_predicate=_pred_gripper_at_pose,
    ),
    "home": SkillSpec(
        name="home",
        description="Return the arm to its nominal home joint configuration with jaws open.",
        requires_target=False,
        parameters={},
        expected_outcome="gripper_at_pose",
        deadline_s=5.0,
        precondition=_precond_no_target,
        verification_predicate=_pred_gripper_at_pose,
    ),
}

SKILL_NAMES: Final[tuple[str, ...]] = tuple(SKILL_REGISTRY.keys())

_FORBIDDEN_COORD_KEYS: Final[frozenset[str]] = frozenset(
    {"x", "y", "z", "target_x", "target_y", "target_z", "coords", "position", "xyz"}
)


def get_skill_spec(name: str) -> SkillSpec:
    """Return the :class:`SkillSpec` for ``name`` or raise :class:`SkillValidationError`."""
    spec = SKILL_REGISTRY.get(name)
    if spec is None:
        raise SkillValidationError(f"unknown skill {name!r}; valid skills: {sorted(SKILL_REGISTRY)}")
    return spec


def validate_skill_call(call: SkillCall, known_target_ids: Set[str]) -> SkillSpec:
    """Validate a :class:`SkillCall` against the registry and current tracked scene IDs."""
    spec = get_skill_spec(call.skill)
    bad_keys = _FORBIDDEN_COORD_KEYS.intersection(call.arguments.keys())
    if bad_keys:
        raise SkillValidationError(
            f"invariant I1 violation: skill {call.skill!r} passed coordinate arguments {sorted(bad_keys)}"
        )
    ok, msg = spec.precondition(call, known_target_ids)
    if not ok:
        raise SkillValidationError(msg)
    return spec


def tool_schemas_for_worker() -> list[dict[str, Any]]:
    """Build OpenAI-compatible function tool schemas for the worker model (invariant I1)."""
    schemas: list[dict[str, Any]] = []
    for spec in SKILL_REGISTRY.values():
        props: dict[str, Any] = {}
        required: list[str] = []
        if spec.requires_target:
            props["target_id"] = {
                "type": "string",
                "description": spec.parameters["target_id"],
            }
            required.append("target_id")
        schemas.append(
            {
                "type": "function",
                "function": {
                    "name": spec.name,
                    "description": spec.description,
                    "parameters": {
                        "type": "object",
                        "properties": props,
                        "required": required,
                        "additionalProperties": False,
                    },
                },
            }
        )
    return schemas
