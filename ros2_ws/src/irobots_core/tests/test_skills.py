"""Tests for the skill library and registry (PLAN:irobots-v2:C10, T10, I1).

Asserts that every skill in the registry has a non-None precondition, a
verification predicate, a valid parameter schema, and that NO skill accepts
a raw Cartesian coordinate (invariant I1).
"""

from __future__ import annotations

import pytest

from irobots_core.skills import (
    SKILL_NAMES,
    SKILL_REGISTRY,
    SkillCall,
    SkillValidationError,
    get_skill_spec,
    tool_schemas_for_worker,
    validate_skill_call,
)


def test_every_skill_has_a_precondition_and_verification_predicate() -> None:
    """T10: registry is total and every skill declares a precondition and predicate."""
    expected = {"move_above", "grasp", "lift", "release", "retreat", "home"}
    assert expected.issubset(set(SKILL_REGISTRY.keys()))
    assert set(SKILL_NAMES) == set(SKILL_REGISTRY.keys())

    for name, spec in SKILL_REGISTRY.items():
        assert spec.name == name
        assert callable(spec.precondition), f"{name} missing callable precondition"
        assert callable(spec.verification_predicate), f"{name} missing callable verification_predicate"
        assert spec.expected_outcome, f"{name} missing expected_outcome"
        assert spec.deadline_s > 0.0, f"{name} must have positive per-skill deadline (I4)"


def test_no_skill_accepts_cartesian_coordinates_invariant_i1() -> None:
    """I1: the worker's tool schema has no coordinate parameters (x, y, z, position, coords)."""
    forbidden_param_names = {"x", "y", "z", "target_x", "target_y", "target_z", "coords", "position", "xyz"}
    for name, spec in SKILL_REGISTRY.items():
        overlap = forbidden_param_names.intersection(spec.parameters.keys())
        assert not overlap, f"skill {name} violates I1 by accepting coordinate params: {overlap}"

    schemas = tool_schemas_for_worker()
    assert len(schemas) == len(SKILL_REGISTRY)
    for schema in schemas:
        props = schema["function"]["parameters"]["properties"]
        assert not forbidden_param_names.intersection(props.keys())


def test_validate_skill_call_rejects_unknown_skill_and_missing_target() -> None:
    """Validating a SkillCall enforces target_id presence and rejects coordinate arguments."""
    valid = SkillCall(skill="move_above", target_id="obj_1")
    validate_skill_call(valid, known_target_ids={"obj_1", "bin_1"})

    # Unknown skill
    with pytest.raises(SkillValidationError, match="unknown skill"):
        validate_skill_call(SkillCall(skill="teleport", target_id="obj_1"), known_target_ids={"obj_1"})

    # Missing target_id on a target-taking skill
    with pytest.raises(SkillValidationError, match="requires a target_id"):
        validate_skill_call(SkillCall(skill="grasp", target_id=None), known_target_ids={"obj_1"})

    # Unknown target_id in scene (D.4 / I1)
    with pytest.raises(SkillValidationError, match="unknown_target"):
        validate_skill_call(SkillCall(skill="grasp", target_id="obj_99"), known_target_ids={"obj_1"})

    # home takes no target_id
    home_call = SkillCall(skill="home", target_id=None)
    validate_skill_call(home_call, known_target_ids=set())
    assert get_skill_spec("home").requires_target is False
