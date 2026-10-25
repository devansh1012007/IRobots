"""First-class thread-safe E-stop interrupt controller (PLAN:irobots-v2:C39, C67-F4, T52, D18).

Unlike the legacy sketch (which polled between ``delay(1000)`` blocks and
dropped PWM to 0, letting the arm fall limp), this controller checks a
thread-safe e-stop event on every single control step, immediately cancels the
trajectory, and holds the last safe pose.
"""

from __future__ import annotations

import threading
from collections.abc import Mapping
from typing import Any


class EStopController:
    """Thread-safe first-class E-stop state machine that halts within one control cycle (T52)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._event = threading.Event()
        self.held_pose: dict[str, float] = {}

    @property
    def triggered(self) -> bool:
        return self._event.is_set()

    @triggered.setter
    def triggered(self, value: bool) -> None:
        if value:
            self._event.set()
        else:
            self._event.clear()

    def trigger(self, hold_pose: Mapping[str, float] | None = None) -> None:
        with self._lock:
            if hold_pose is not None:
                self.held_pose = {str(k): float(v) for k, v in hold_pose.items()}
            self._event.set()

    def reset(self) -> None:
        with self._lock:
            self._event.clear()
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
            with self._lock:
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
                with self._lock:
                    self.held_pose = sim.joint_positions()
                    sim.set_joint_targets(self.held_pose)
                return (taken, True)
            sim.step(1)
            taken += 1
        return (taken, False)
