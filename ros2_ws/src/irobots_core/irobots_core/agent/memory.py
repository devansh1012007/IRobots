"""Per-episode working memory (PLAN:irobots-v2:C33, T44).

Tracks what skills were tried, their outcomes, and the current hypothesis within
an episode. Strictly scoped to a single episode so state never leaks across
trials.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..skills import SkillCall


@dataclass(frozen=True, slots=True)
class MemoryEntry:
    turn_index: int
    call: SkillCall
    outcome: str
    success: bool


class EpisodeMemory:
    """Per-episode memory of attempted skills, failures, and failure hypothesis."""

    def __init__(self, episode_id: str = "ep_0") -> None:
        self.episode_id = episode_id
        self.history: list[MemoryEntry] = []
        self.current_hypothesis: str = ""

    def reset(self, new_episode_id: str) -> None:
        self.episode_id = new_episode_id
        self.history.clear()
        self.current_hypothesis = ""

    def record_step(
        self,
        call: SkillCall,
        *,
        outcome: str,
        success: bool,
        hypothesis: str | None = None,
    ) -> MemoryEntry:
        entry = MemoryEntry(
            turn_index=len(self.history),
            call=call,
            outcome=outcome,
            success=success,
        )
        self.history.append(entry)
        if hypothesis is not None:
            self.current_hypothesis = hypothesis
        return entry

    def failures(self) -> list[MemoryEntry]:
        return [e for e in self.history if not e.success]

    def consecutive_repeats_of(self, call: SkillCall) -> int:
        """Count how many trailing turns executed the exact same ``(skill, target_id)``."""
        count = 0
        for entry in reversed(self.history):
            if entry.call.skill == call.skill and entry.call.target_id == call.target_id:
                count += 1
            else:
                break
        return count
