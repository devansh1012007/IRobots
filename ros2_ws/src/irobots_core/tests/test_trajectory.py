"""Tests for trapezoidal joint-space trajectories (PLAN:irobots-v2:C08, T06).

Asserts monotonicity, velocity and acceleration limit compliance, and that
duration is a pure deterministic function of the inputs (never wall-clock).
"""

from __future__ import annotations

import math

import pytest

from irobots_core.trajectory import JointTrajectory, plan_joint_trajectory


def test_trajectory_is_monotonic_and_respects_limits(robot) -> None:
    """T06: profile is monotonic per joint and stays within velocity/acceleration limits."""
    start = {k: float(robot.home_pose[k]) for k in robot.joints}
    goal = {
        "base_yaw": 0.8,
        "shoulder_pitch": 0.5,
        "elbow_pitch": -0.6,
        "wrist_pitch": 0.2,
        "wrist_yaw": -1.1,
    }
    traj = plan_joint_trajectory(robot, start, goal)
    assert isinstance(traj, JointTrajectory)
    assert traj.duration > 0.0

    # Sample at 200 points and check monotonicity and velocity/accel bounds
    n = 200
    times = [traj.duration * i / n for i in range(n + 1)]
    samples = [traj.sample(t) for t in times]

    assert samples[0].positions == pytest.approx(start, abs=1e-9)
    assert samples[-1].positions == pytest.approx(goal, abs=1e-9)

    for jname, jspec in robot.joints.items():
        s0 = start[jname]
        s1 = goal[jname]
        direction = 1.0 if s1 >= s0 else -1.0
        prev_p = s0
        for pt in samples:
            p = pt.positions[jname]
            v = pt.velocities[jname]
            a = pt.accelerations[jname]
            # Monotonic progress toward goal
            assert (p - prev_p) * direction >= -1e-9, f"non-monotonic step on {jname}"
            prev_p = p
            assert abs(v) <= jspec.max_velocity + 1e-6, f"{jname} velocity {v} exceeds {jspec.max_velocity}"
            assert abs(a) <= jspec.max_acceleration + 1e-6, f"{jname} accel {a} exceeds {jspec.max_acceleration}"


def test_trajectory_duration_is_a_pure_function_of_inputs(robot) -> None:
    """T06 / I6: duration depends only on start, goal, and limits — never wall-clock."""
    start = {k: float(robot.home_pose[k]) for k in robot.joints}
    goal = {
        "base_yaw": -0.6,
        "shoulder_pitch": 0.4,
        "elbow_pitch": 0.7,
        "wrist_pitch": -0.3,
        "wrist_yaw": 0.5,
    }
    t1 = plan_joint_trajectory(robot, start, goal)
    t2 = plan_joint_trajectory(robot, start, goal)
    assert t1.duration == t2.duration
    assert math.isfinite(t1.duration)
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        s1 = t1.sample(frac * t1.duration)
        s2 = t2.sample(frac * t2.duration)
        assert s1.positions == s2.positions
        assert s1.velocities == s2.velocities


def test_zero_displacement_trajectory_has_zero_duration(robot) -> None:
    """Edge case: identical start and goal yields a valid zero-duration profile."""
    pose = dict(robot.home_pose)
    traj = plan_joint_trajectory(robot, pose, pose)
    assert traj.duration == 0.0
    pt = traj.sample(0.0)
    for jname in robot.joints:
        assert pt.positions[jname] == pytest.approx(pose[jname])
        assert pt.velocities[jname] == 0.0
