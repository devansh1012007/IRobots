"""Single-step worker agent enforcing Invariant I3 and repeat detection (PLAN:irobots-v2:C32, T41, T42, T43).

Given the current scene, approved plan, and episode memory:
1. Emits strictly ONE skill call per turn (I3).
2. Refuses any target_id not present in the tracker's active scene IDs (T43).
3. Escalates via :class:`RepeatActionEscalation` when the same ``(skill, target_id)``
   has already been attempted ``max_repeats`` times in a row (T42).
"""

from __future__ import annotations

from collections.abc import Sequence, Set

from ..skills import SkillCall, validate_skill_call
from .memory import EpisodeMemory


class RepeatActionEscalation(RuntimeError):
    """Raised when the worker detects a repeated skill+target loop and escalates to replan."""


class WorkerAgent:
    """Executes one validated skill call per observation (I1, I3)."""

    def __init__(self, *, max_repeats: int = 2) -> None:
        self.max_repeats = max_repeats

    def select_next_skill(
        self,
        plan: Sequence[SkillCall],
        scene_target_ids: Set[str],
        memory: EpisodeMemory,
    ) -> SkillCall:
        if not plan:
            return SkillCall(skill="home")

        # Invariant I3: even if a caller passes a multi-step plan, emit only the single next skill
        candidate = plan[0]
        validate_skill_call(candidate, scene_target_ids)

        repeats = memory.consecutive_repeats_of(candidate)
        if repeats >= self.max_repeats:
            raise RepeatActionEscalation(
                f"skill {candidate.skill!r} on target {candidate.target_id!r} repeated {repeats} times"
            )
        return candidate
