"""Task planner and scene-grounded plan approval (PLAN:irobots-v2:C31, T40).

Validates every proposed plan step against the skill registry and the set of
currently tracked object IDs in the scene BEFORE approval. A plan referencing
a non-existent object is rejected immediately rather than entering an infinite
worker execution loop.
"""

from __future__ import annotations

from collections.abc import Sequence, Set

from ..skills import SkillCall, SkillValidationError, validate_skill_call


class PlanApprovalError(ValueError):
    """Raised when a proposed plan references an unknown skill or absent scene target."""


class TaskPlanner:
    """Produces and validates ordered skill plans without Cartesian coordinates (I1)."""

    def plan_for_task(
        self,
        goal_type: str,
        target_id: str,
        *,
        bin_id: str | None = None,
    ) -> list[SkillCall]:
        """Generate the nominal skill plan for ``lift`` (L1) or ``place_in_bin`` (L2)."""
        if goal_type == "lift":
            return [
                SkillCall(skill="move_above", target_id=target_id),
                SkillCall(skill="grasp", target_id=target_id),
                SkillCall(skill="lift", target_id=target_id),
            ]
        if goal_type == "place_in_bin":
            if not bin_id:
                raise PlanApprovalError("place_in_bin requires a bin_id")
            return [
                SkillCall(skill="move_above", target_id=target_id),
                SkillCall(skill="grasp", target_id=target_id),
                SkillCall(skill="lift", target_id=target_id),
                SkillCall(skill="move_above", target_id=bin_id),
                SkillCall(skill="release"),
                SkillCall(skill="retreat"),
            ]
        raise PlanApprovalError(f"unsupported goal_type {goal_type!r}")

    def validate_and_approve(
        self,
        proposed_steps: Sequence[SkillCall],
        scene_target_ids: Set[str],
    ) -> list[SkillCall]:
        """Validate every step in ``proposed_steps`` against ``scene_target_ids`` (T40)."""
        if not proposed_steps:
            raise PlanApprovalError("proposed plan is empty")
        approved: list[SkillCall] = []
        for idx, step in enumerate(proposed_steps):
            try:
                validate_skill_call(step, scene_target_ids)
            except SkillValidationError as exc:
                raise PlanApprovalError(f"plan step {idx} rejected: {exc}") from exc
            approved.append(step)
        return approved
