"""Tests for the analytic kinematics (PLAN:irobots-v2:C07).

Two properties matter and both are silent when they break:

1. The forward and inverse solvers must agree with each other, and
2. the analytic solver must agree with the *simulator*, which is an
   independent implementation of the same chain.

A mismatch between the analytic model and the simulated model is the worst
kind of bug in this project: the arm still moves, the episodes still run, and
every number is quietly wrong.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from irobots_core.kinematics import (
    IKStatus,
    forward_kinematics,
    inverse_kinematics,
    workspace_reach_check,
)

TOL = 1e-6


def _workspace_samples(robot, n: int = 60):
    cx, cy = robot.safety.workspace_centre
    radius = robot.safety.workspace_radius
    rng = np.random.default_rng(7)
    rr = radius * np.sqrt(rng.random(n))
    th = rng.random(n) * 2.0 * math.pi
    return [(float(cx + r * math.cos(t)), float(cy + r * math.sin(t))) for r, t in zip(rr, th, strict=True)]


def test_fk_of_ik_is_the_target(robot) -> None:
    """T03: the round trip is exact, not approximate."""
    for x, y in _workspace_samples(robot):
        for z in (0.02, 0.06):
            res = inverse_kinematics(robot, (x, y, z), closing_azimuth=0.0)
            assert res.status is IKStatus.REACHABLE, res.message
            state = forward_kinematics(robot, res.pose)
            error = np.linalg.norm(state.position - np.array([x, y, z]))
            assert error < TOL, f"round trip off by {error:.3e} m at {(x, y, z)}"


def test_the_closing_direction_is_the_one_asked_for(robot) -> None:
    """The wrist yaw must actually set the jaw direction, not saturate."""
    x, y = _workspace_samples(robot)[0]
    for azimuth in (0.0, math.pi / 2, math.pi, -math.pi / 2):
        res = inverse_kinematics(robot, (x, y, 0.02), closing_azimuth=azimuth)
        if not res.ok:
            continue
        state = forward_kinematics(robot, res.pose)
        error = abs((state.azimuth - azimuth + math.pi) % (2 * math.pi) - math.pi)
        assert error < math.radians(1.0), (
            f"closing direction off by {math.degrees(error):.2f} deg"
        )


def test_an_unreachable_target_is_reported_not_clamped(robot) -> None:
    """T04: the caller must be able to tell."""
    res = inverse_kinematics(robot, (1.5, 0.0, 0.05))
    assert res.status is IKStatus.UNREACHABLE
    assert res.closest is not None
    assert np.all(np.isfinite(res.closest))
    assert np.linalg.norm(res.closest) < 1.5


def test_the_singularity_on_the_base_axis_is_reported(robot) -> None:
    """T05: a target on the axis has no defined azimuth."""
    res = inverse_kinematics(robot, (0.0, 0.0, 0.05))
    assert res.status is IKStatus.SINGULAR


def test_a_reachable_answer_never_violates_a_joint_limit(robot) -> None:
    """A pose outside the limits must never be presented as reachable.

    This is the property that matters. A solver that returns REACHABLE with
    an out-of-range joint is a false positive that the caller cannot detect,
    and the arm then stalls short of the target for reasons that look like a
    control problem.
    """
    for x, y in _workspace_samples(robot, 25):
        for z in (0.0, 0.02, 0.06, 0.12):
            res = inverse_kinematics(robot, (x, y, z))
            if res.status is not IKStatus.REACHABLE:
                continue
            for name, value in res.pose.items():
                spec = robot.joint(name)
                assert spec.min_rad - 1e-9 <= value <= spec.max_rad + 1e-9, (
                    f"{name}={value} is outside its limits but the pose was called reachable"
                )


def test_a_target_inside_the_minimum_fold_radius_is_unreachable(robot) -> None:
    """The arm cannot fold tighter than |L1 - L2|."""
    res = inverse_kinematics(robot, (0.0, 0.02, 0.02))
    assert res.status is IKStatus.UNREACHABLE
    assert res.closest is not None
    assert np.all(np.isfinite(res.closest))


def test_ik_never_returns_nan(robot) -> None:
    for x, y in _workspace_samples(robot, 30):
        for z in (-0.05, 0.0, 0.02, 0.06, 0.5):
            res = inverse_kinematics(robot, (x, y, z))
            for value in res.pose.values():
                assert math.isfinite(value)
            if res.closest is not None:
                assert np.all(np.isfinite(res.closest))


def test_the_whole_workspace_is_reachable(robot) -> None:
    """The workspace is chosen so that every point in it is a legal target."""
    unreachable = [p for p, _y, ok in workspace_reach_check(robot, 120) if not ok]
    assert not unreachable, f"{len(unreachable)} workspace points are unreachable"


def test_the_analytic_model_matches_the_simulator(robot, scene) -> None:
    """The two implementations of the same chain must agree exactly.

    This is the test that caught five silent kinematics bugs: joint limits in
    the wrong unit, a fixed elbow branch, a discontinuous wrist-yaw solve,
    joints attached to the wrong bodies in the MJCF, and adjacent links
    overlapping at the joints so the arm fought itself.
    """
    mujoco = pytest.importorskip("mujoco")
    from irobots_core.sim.mujoco_sim import MuJoCoSim

    sim = MuJoCoSim(robot, scene, [])
    rng = np.random.default_rng(0)
    max_pos = 0.0
    max_dir = 0.0
    for _ in range(60):
        pose = {name: float(rng.uniform(-1.0, 1.0)) for name in robot.joints}
        for name, jid in sim._arm_joint_ids.items():
            sim.data.qpos[sim.mj.jnt_qposadr[jid]] = pose[name]
        mujoco.mj_forward(sim.mj, sim.data)

        state = forward_kinematics(robot, sim.joint_positions())
        max_pos = max(max_pos, float(np.linalg.norm(state.position - sim.tool_position())))
        max_dir = max(
            max_dir,
            float(np.linalg.norm(state.closing - sim.tool_matrix() @ np.array([1.0, 0.0, 0.0]))),
        )

    assert max_pos < TOL, f"tip position differs by {max_pos:.3e} m"
    assert max_dir < TOL, f"closing direction differs by {max_dir:.3e}"
