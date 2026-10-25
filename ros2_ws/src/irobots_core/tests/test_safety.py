"""Tests for the safety supervisor (PLAN:irobots-v2:C09, T07, T08, T09, I2).

The supervisor is a pure function between a confused model and a broken arm.
It must reject out-of-bounds poses, table-plane violations, self-collisions,
joint-limit violations, speed-limit violations, and NaN/Inf inputs.
"""

from __future__ import annotations

import math

from irobots_core.kinematics import inverse_kinematics
from irobots_core.safety import (
    SafetyReason,
    SafetySupervisor,
    check_cartesian_target,
    check_pose,
    preflight_check_plan,
)


def test_valid_workspace_pose_is_approved(robot) -> None:
    """A nominal pose inside the workspace and above the table is approved."""
    cx, cy = robot.safety.workspace_centre
    ik = inverse_kinematics(robot, (cx, cy, 0.04))
    assert ik.ok
    verdict = check_pose(robot, ik.pose, opening=0.045)
    assert verdict.approved
    assert verdict.reason is SafetyReason.OK


def test_every_out_of_bounds_pose_is_rejected(robot) -> None:
    """T07: poses outside the workspace disk, z_max, or joint limits are rejected."""
    sup = SafetySupervisor(robot)

    # 1. Cartesian target outside the workspace disk (radius 0.10 around (0, 0.20))
    v_xy = check_cartesian_target(robot, (0.18, 0.20, 0.04))
    assert not v_xy.approved
    assert v_xy.reason is SafetyReason.WORKSPACE_BOUNDS

    # 2. Cartesian target above z_max (0.30 m)
    cx, cy = robot.safety.workspace_centre
    v_z = check_cartesian_target(robot, (cx, cy, 0.38))
    assert not v_z.approved
    assert v_z.reason is SafetyReason.WORKSPACE_BOUNDS

    # 3. Joint pose whose FK tip lies outside the workspace disk
    ik_far = inverse_kinematics(robot, (0.18, 0.25, 0.06))
    assert ik_far.ok
    v_pose = sup.check(ik_far.pose)
    assert not v_pose.approved
    assert v_pose.reason is SafetyReason.WORKSPACE_BOUNDS

    # 4. Joint limit violation
    bad_joint = dict(robot.home_pose)
    bad_joint["base_yaw"] = math.radians(175.0)  # limit is +/-170 deg
    v_jl = sup.check(bad_joint)
    assert not v_jl.approved
    assert v_jl.reason is SafetyReason.JOINT_LIMIT

    # 5. Gripper limit violation
    ok_ik = inverse_kinematics(robot, (cx, cy, 0.04))
    v_grip = sup.check(ok_ik.pose, opening=0.095)  # max is 0.070 m
    assert not v_grip.approved
    assert v_grip.reason is SafetyReason.GRIPPER_LIMIT

    # 6. Speed limit violation
    v_spd = sup.check(ok_ik.pose, velocities={"base_yaw": 9.9})
    assert not v_spd.approved
    assert v_spd.reason is SafetyReason.SPEED_LIMIT


def test_table_plane_violation_is_rejected(robot) -> None:
    """T08: any link or tool point penetrating below table_plane_z + clearance is rejected."""
    cx, cy = robot.safety.workspace_centre
    # Tool tip at z = 0.0005 m is below table_plane_z (0.0) + table_clearance (0.002 m)
    ik_low = inverse_kinematics(robot, (cx, cy, 0.0005))
    assert ik_low.ok
    verdict = check_pose(robot, ik_low.pose)
    assert not verdict.approved
    assert verdict.reason is SafetyReason.TABLE_PLANE

    # Also check an elbow-down configuration where an intermediate joint hits the table
    v_cart = check_cartesian_target(robot, (cx, cy, -0.01))
    assert not v_cart.approved
    assert v_cart.reason is SafetyReason.TABLE_PLANE


def test_no_nan_or_inf_ever_passes(robot) -> None:
    """T09: NaN or Inf in any joint, opening, velocity, or Cartesian target is rejected."""
    cx, cy = robot.safety.workspace_centre
    ik = inverse_kinematics(robot, (cx, cy, 0.04))
    for bad_val in (float("nan"), float("inf"), float("-inf")):
        pose_bad = dict(ik.pose)
        pose_bad["elbow_pitch"] = bad_val
        v1 = check_pose(robot, pose_bad)
        assert not v1.approved
        assert v1.reason is SafetyReason.NON_FINITE

        v2 = check_pose(robot, ik.pose, opening=bad_val)
        assert not v2.approved
        assert v2.reason is SafetyReason.NON_FINITE

        v3 = check_pose(robot, ik.pose, velocities={"shoulder_pitch": bad_val})
        assert not v3.approved
        assert v3.reason is SafetyReason.NON_FINITE

        v4 = check_cartesian_target(robot, (bad_val, cy, 0.04))
        assert not v4.approved
        assert v4.reason is SafetyReason.NON_FINITE


def test_self_collision_near_base_column_is_rejected(robot) -> None:
    """Self-collision check rejects tool or wrist folding into the base column."""
    # Fold the arm back into the base column (r < self_collision_radius = 0.06 m)
    folded = {
        "base_yaw": 0.0,
        "shoulder_pitch": math.radians(85.0),
        "elbow_pitch": math.radians(-155.0),
        "wrist_pitch": math.radians(-20.0),
        "wrist_yaw": 0.0,
    }
    verdict = check_pose(robot, folded)
    assert not verdict.approved
    assert verdict.reason in (SafetyReason.SELF_COLLISION, SafetyReason.WORKSPACE_BOUNDS)


def test_preflight_feasibility_check_counts_infeasible_steps(robot) -> None:
    """Edge case (§I): pre-flight check of a nominal plan catches infeasible waypoints before execution."""
    cx, cy = robot.safety.workspace_centre
    ok_ik = inverse_kinematics(robot, (cx, cy, 0.04))
    bad_ik = inverse_kinematics(robot, (0.20, 0.25, 0.04))
    verdicts, rejected_infeasible = preflight_check_plan(robot, [ok_ik.pose, bad_ik.pose])
    assert len(verdicts) == 2
    assert verdicts[0].approved
    assert not verdicts[1].approved
    assert rejected_infeasible == 1
