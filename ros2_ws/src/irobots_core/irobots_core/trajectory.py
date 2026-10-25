"""Deterministic trapezoidal joint-space trajectory profiles.

The legacy firmware snapped angles directly and used ``delay()``, which swept
objects off the table and coupled trajectory timing to wall-clock sleep.
Here, every trajectory duration is a pure function of the joint displacements
and the configured velocity/acceleration limits (invariant I6).

PLAN:irobots-v2:C08
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from .robot_model import GRIPPER_CHANNEL, RobotModel


@dataclass(frozen=True, slots=True)
class ScalarTrapezoid:
    """One 1D trapezoidal (or triangular) velocity profile."""

    start: float
    goal: float
    v_max: float
    a_max: float
    t_accel: float
    t_cruise: float
    duration: float
    peak_v: float

    @classmethod
    def build(cls, start: float, goal: float, v_max: float, a_max: float) -> ScalarTrapezoid:
        if v_max <= 0.0 or a_max <= 0.0:
            raise ValueError(f"v_max and a_max must be positive, got {v_max}, {a_max}")
        dist = abs(goal - start)
        if dist < 1e-12:
            return cls(start, goal, v_max, a_max, 0.0, 0.0, 0.0, 0.0)
        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc * t_acc
        if 2.0 * d_acc >= dist:
            # Triangular profile: never reaches v_max
            t_acc = math.sqrt(dist / a_max)
            t_cruise = 0.0
            peak_v = a_max * t_acc
        else:
            d_cruise = dist - 2.0 * d_acc
            t_cruise = d_cruise / v_max
            peak_v = v_max
        duration = 2.0 * t_acc + t_cruise
        return cls(start, goal, v_max, a_max, t_acc, t_cruise, duration, peak_v)

    def sample(self, t: float, total_duration: float) -> tuple[float, float, float]:
        """Evaluate (position, velocity, acceleration) at time ``t``.

        When synchronized across multiple joints to ``total_duration``, time is
        scaled uniformly so all joints start and finish together without
        exceeding their individual velocity or acceleration bounds.
        """
        dist = self.goal - self.start
        if abs(dist) < 1e-12 or self.duration <= 0.0 or total_duration <= 0.0:
            return (self.goal, 0.0, 0.0)
        if t <= 0.0:
            return (self.start, 0.0, 0.0)
        if t >= total_duration:
            return (self.goal, 0.0, 0.0)

        scale = self.duration / total_duration
        tau = t * scale
        sign = 1.0 if dist >= 0.0 else -1.0
        t_a = self.t_accel
        t_c = self.t_cruise
        a = self.a_max
        v_p = self.peak_v

        if tau <= t_a:
            s = 0.5 * a * tau * tau
            v = a * tau
            acc = a
        elif tau <= t_a + t_c:
            dt = tau - t_a
            s = 0.5 * a * t_a * t_a + v_p * dt
            v = v_p
            acc = 0.0
        else:
            dt = self.duration - tau
            s = abs(dist) - 0.5 * a * dt * dt
            v = a * dt
            acc = -a

        return (
            self.start + sign * s,
            sign * v * scale,
            sign * acc * scale * scale,
        )


@dataclass(frozen=True, slots=True)
class TrajectoryPoint:
    """State of all joints at a single timestamp along a trajectory."""

    time_s: float
    positions: dict[str, float]
    velocities: dict[str, float]
    accelerations: dict[str, float]


@dataclass(frozen=True, slots=True)
class JointTrajectory:
    """Synchronized multi-joint trapezoidal trajectory."""

    profiles: Mapping[str, ScalarTrapezoid]
    duration: float

    def sample(self, t: float) -> TrajectoryPoint:
        clamped_t = max(0.0, min(self.duration, float(t)))
        pos: dict[str, float] = {}
        vel: dict[str, float] = {}
        acc: dict[str, float] = {}
        for name, prof in self.profiles.items():
            p, v, a = prof.sample(clamped_t, self.duration)
            pos[name] = p
            vel[name] = v
            acc[name] = a
        return TrajectoryPoint(
            time_s=clamped_t,
            positions=pos,
            velocities=vel,
            accelerations=acc,
        )

    def discretize(self, dt: float) -> Sequence[TrajectoryPoint]:
        """Discretize the trajectory at a fixed timestep ``dt`` (invariant I6)."""
        if dt <= 0.0:
            raise ValueError(f"dt must be positive, got {dt}")
        if self.duration <= 0.0:
            return [self.sample(0.0)]
        steps = max(1, math.ceil(self.duration / dt))
        return [self.sample(min(self.duration, i * dt)) for i in range(steps + 1)]


def plan_joint_trajectory(
    model: RobotModel,
    start: Mapping[str, float],
    goal: Mapping[str, float],
) -> JointTrajectory:
    """Plan a synchronized trapezoidal trajectory from ``start`` to ``goal``."""
    profiles: dict[str, ScalarTrapezoid] = {}
    max_duration = 0.0

    for name, spec in model.joints.items():
        s0 = float(start.get(name, model.home_pose.get(name, 0.0)))
        s1 = float(goal.get(name, s0))
        if not (math.isfinite(s0) and math.isfinite(s1)):
            raise ValueError(f"non-finite joint angle for {name!r}: {s0} -> {s1}")
        prof = ScalarTrapezoid.build(s0, s1, spec.max_velocity, spec.max_acceleration)
        profiles[name] = prof
        if prof.duration > max_duration:
            max_duration = prof.duration

    if GRIPPER_CHANNEL in start or GRIPPER_CHANNEL in goal:
        g0 = float(start.get(GRIPPER_CHANNEL, model.gripper.max_opening))
        g1 = float(goal.get(GRIPPER_CHANNEL, g0))
        if not (math.isfinite(g0) and math.isfinite(g1)):
            raise ValueError(f"non-finite gripper opening: {g0} -> {g1}")
        gprof = ScalarTrapezoid.build(
            g0, g1, model.gripper.max_velocity, model.gripper.max_acceleration
        )
        profiles[GRIPPER_CHANNEL] = gprof
        if gprof.duration > max_duration:
            max_duration = gprof.duration

    return JointTrajectory(profiles=profiles, duration=max_duration)
