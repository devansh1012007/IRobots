"""First-class E-stop interrupt controller (PLAN:irobots-v2:C39, T52, D18).

Unlike the legacy sketch (which polled between ``delay(1000)`` blocks and
dropped PWM to 0, letting the arm fall limp), this controller checks the
e-stop flag on every single control step, immediately cancels the trajectory,
and holds the last safe pose.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any


class EStopController:
    """First-class E-stop state machine that halts within one control cycle (T52)."""

    def __init__(self) -> None:
        self.triggered: bool = False
        self.held_pose: dict[str, float] = {}

    def trigger(self, hold_pose: Mapping[str, float]) -> None:
        self.triggered = True
        self.held_pose = {str(k): float(v) for k, v in hold_pose.items()}

    def reset(self) -> None:
        self.triggered = False
        self.held_pose.clear()

    def step_guarded(
        self,
        sim: Any,
        *,
        goal_pose: Mapping[str, float],
        max_steps: int = 400,
    ) -> tuple[int, bool]:
        """Step ``sim`` toward ``goal_pose``, aborting within 1 cycle if e-stopped."""
        if self.triggered:
            if self.held_pose:
                sim.set_joint_targets(self.held_pose)
            else:
                self.held_pose = sim.joint_positions()
                sim.set_joint_targets(self.held_pose)
            sim.step(1)
            return (1, True)

        sim.set_joint_targets(goal_pose)
        taken = 0
        for _ in range(max_steps):
            if self.triggered:
                self.held_pose = sim.joint_positions()
                sim.set_joint_targets(self.held_pose)
                return (taken, True)
            sim.step(1)
            taken += 1
        return (taken, False)
