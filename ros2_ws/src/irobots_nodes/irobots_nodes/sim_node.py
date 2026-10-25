"""Simulator wrapper node enforcing use_sim_time and pause-on-think (PLAN:irobots-v2:C23, I6).

Publishes JointState and Image snapshots stamped with the simulated clock
``sim.total_steps * sim.dt`` and holds the simulator paused while the LLM or
perception pipeline is thinking.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True, slots=True)
class SimImageStamp:
    timestamp_s: float
    rgb: np.ndarray


@dataclass(frozen=True, slots=True)
class SimJointStateStamp:
    timestamp_s: float
    positions: dict[str, float]
    opening_m: float


class SimNode:
    """Thin simulator node owning pause/step discipline and sim-time stamping (C23)."""

    def __init__(self, sim: Any, *, use_sim_time: bool = True) -> None:
        if not use_sim_time:
            raise ValueError("Invariant I6 requires use_sim_time=True on SimNode")
        self.sim = sim
        self.use_sim_time = use_sim_time
        self.paused: bool = False
        self._cached_frame: SimImageStamp | None = None

    @property
    def sim_time_s(self) -> float:
        return float(self.sim.total_steps * self.sim.dt)

    def pause(self) -> None:
        self.paused = True

    def resume(self) -> None:
        self.paused = False

    def step_sim(self, n_steps: int = 1) -> float:
        if self.paused:
            return self.sim_time_s
        self.sim.step(n_steps)
        self._cached_frame = None
        return self.sim_time_s

    def capture_image(self) -> SimImageStamp:
        """Capture an RGB frame stamped with the exact simulation time (T30, T31)."""
        t_now = self.sim_time_s
        if self._cached_frame is not None and self._cached_frame.timestamp_s == t_now:
            return SimImageStamp(timestamp_s=t_now, rgb=self._cached_frame.rgb.copy())
        rgb = self.sim.render().copy()
        self._cached_frame = SimImageStamp(timestamp_s=t_now, rgb=rgb)
        return SimImageStamp(timestamp_s=t_now, rgb=rgb.copy())

    def read_joint_state(self) -> SimJointStateStamp:
        return SimJointStateStamp(
            timestamp_s=self.sim_time_s,
            positions=self.sim.joint_positions(),
            opening_m=self.sim.opening(),
        )
