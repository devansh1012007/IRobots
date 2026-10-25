"""The safety supervisor: a pure function between a confused model and the arm.

Invariant I2: every commanded pose passes the safety supervisor before
execution. A rejection is a first-class outcome the planner sees, never an
uncaught exception or a silent clamp.

Checks enforced on every pose:
1. Finite numbers (no NaN or Inf in joints, gripper opening, velocities, or target).
2. Joint limits for all 5 revolute joints and the prismatic gripper opening.
3. Velocity limits when velocities are supplied.
4. Link and tool clearance above the table plane (checks elbow, wrist, and tool
   tip — not just the TCP, per PLAN.md §I).
5. Self-collision cylinder around the base column.
6. Workspace cylinder bounds (disk radius and z_min..z_max).

PLAN:irobots-v2:C09
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum

import numpy as np

from .kinematics import forward_kinematics
from .robot_model import GRIPPER_CHANNEL, JOINT_ORDER, RobotModel


class SafetyReason(StrEnum):
    """Why a pose or target was approved or rejected."""

    OK = "ok"
    NON_FINITE = "non_finite"
    UNKNOWN_JOINT = "unknown_joint"
    JOINT_LIMIT = "joint_limit"
    GRIPPER_LIMIT = "gripper_limit"
    SPEED_LIMIT = "speed_limit"
    TABLE_PLANE = "table_plane"
    SELF_COLLISION = "self_collision"
    WORKSPACE_BOUNDS = "workspace_bounds"


@dataclass(frozen=True, slots=True)
class Verdict:
    """First-class safety supervisor decision (invariant I2)."""

    approved: bool
    reason: SafetyReason
    detail: str = ""


def _chain_keypoints(model: RobotModel, pose: Mapping[str, float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute world positions of (elbow, wrist, tip) for link-vs-table and self-collision checks."""
    theta_s = float(pose.get("shoulder_pitch", 0.0))
    theta_e = float(pose.get("elbow_pitch", 0.0))
    psi_b = float(pose.get("base_yaw", 0.0))

    l1 = float(model.upper_arm)
    l2 = float(model.fore_arm)
    base_h = float(model.base_height)

    phi1 = theta_s
    phi2 = phi1 + theta_e

    c_b, s_b = math.cos(psi_b), math.sin(psi_b)
    r_base = np.array([[c_b, -s_b, 0.0], [s_b, c_b, 0.0], [0.0, 0.0, 1.0]])

    d1 = np.array([math.cos(phi1), 0.0, math.sin(phi1)])
    d2 = np.array([math.cos(phi2), 0.0, math.sin(phi2)])

    shoulder = np.array([0.0, 0.0, base_h])
    elbow = shoulder + r_base @ (l1 * d1)
    wrist = elbow + r_base @ (l2 * d2)
    tip = forward_kinematics(model, pose).position
    return elbow, wrist, tip


def check_cartesian_target(
    model: RobotModel,
    target_xyz: Sequence[float],
) -> Verdict:
    """Check whether a Cartesian target point lies inside the safe workspace above the table."""
    if len(target_xyz) != 3 or not all(math.isfinite(float(v)) for v in target_xyz):
        return Verdict(False, SafetyReason.NON_FINITE, f"non-finite target {target_xyz!r}")

    x, y, z = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])
    s = model.safety
    min_z = s.table_plane_z + s.table_clearance
    if z < min_z - 1e-9:
        return Verdict(
            False,
            SafetyReason.TABLE_PLANE,
            f"target z={z:.4f}m is below table clearance {min_z:.4f}m",
        )
    if z < s.z_min - 1e-9 or z > s.z_max + 1e-9:
        return Verdict(
            False,
            SafetyReason.WORKSPACE_BOUNDS,
            f"target z={z:.4f}m outside [{s.z_min:.3f}, {s.z_max:.3f}]m",
        )
    cx, cy = s.workspace_centre
    dist_xy = math.hypot(x - cx, y - cy)
    if dist_xy > s.workspace_radius + 1e-9:
        return Verdict(
            False,
            SafetyReason.WORKSPACE_BOUNDS,
            f"target (x={x:.3f}, y={y:.3f}) is {dist_xy:.3f}m from workspace centre, radius={s.workspace_radius:.3f}m",
        )
    if math.hypot(x, y) < s.self_collision_radius - 1e-9:
        return Verdict(
            False,
            SafetyReason.SELF_COLLISION,
            f"target radial distance {math.hypot(x, y):.3f}m inside base column radius {s.self_collision_radius:.3f}m",
        )
    return Verdict(True, SafetyReason.OK, "ok")


def check_pose(
    model: RobotModel,
    pose: Mapping[str, float],
    *,
    opening: float | None = None,
    velocities: Mapping[str, float] | None = None,
    enforce_workspace: bool = True,
) -> Verdict:
    """Validate a joint pose against all safety invariants.

    Never raises on a bad pose and never acts as a no-op path.
    """
    for jname in JOINT_ORDER:
        if jname not in pose:
            return Verdict(False, SafetyReason.UNKNOWN_JOINT, f"missing joint {jname!r}")

    for name, val in pose.items():
        if name == GRIPPER_CHANNEL:
            if opening is None:
                opening = val
            continue
        if name not in model.joints:
            return Verdict(False, SafetyReason.UNKNOWN_JOINT, f"unknown joint {name!r}")
        if not isinstance(val, (int, float)) or not math.isfinite(float(val)):
            return Verdict(False, SafetyReason.NON_FINITE, f"joint {name!r} is non-finite: {val!r}")

    if opening is not None:
        if not isinstance(opening, (int, float)) or not math.isfinite(float(opening)):
            return Verdict(False, SafetyReason.NON_FINITE, f"gripper opening is non-finite: {opening!r}")
        g = model.gripper
        if float(opening) < g.min_opening - 1e-9 or float(opening) > g.max_opening + 1e-9:
            return Verdict(
                False,
                SafetyReason.GRIPPER_LIMIT,
                f"gripper opening {float(opening):.4f}m outside [{g.min_opening:.4f}, {g.max_opening:.4f}]m",
            )

    if velocities is not None:
        for name, v in velocities.items():
            if not isinstance(v, (int, float)) or not math.isfinite(float(v)):
                return Verdict(False, SafetyReason.NON_FINITE, f"velocity for {name!r} is non-finite: {v!r}")
            if name == GRIPPER_CHANNEL:
                max_v = model.gripper.max_velocity
            elif name in model.joints:
                max_v = model.joint(name).max_velocity
            else:
                return Verdict(False, SafetyReason.UNKNOWN_JOINT, f"unknown velocity joint {name!r}")
            if abs(float(v)) > max_v + 1e-6:
                return Verdict(
                    False,
                    SafetyReason.SPEED_LIMIT,
                    f"joint {name!r} speed {abs(float(v)):.3f} exceeds {max_v:.3f}",
                )

    for jname in JOINT_ORDER:
        spec = model.joint(jname)
        angle = float(pose[jname])
        if angle < spec.min_rad - 1e-9 or angle > spec.max_rad + 1e-9:
            return Verdict(
                False,
                SafetyReason.JOINT_LIMIT,
                f"joint {jname!r}={math.degrees(angle):.2f} deg outside [{spec.min_deg:.1f}, {spec.max_deg:.1f}] deg",
            )

    elbow, wrist, tip = _chain_keypoints(model, pose)
    s = model.safety
    min_z = s.table_plane_z + s.table_clearance

    # Check every link keypoint against the table plane, not just the TCP (PLAN.md §I)
    for label, pt in (("elbow", elbow), ("wrist", wrist), ("tip", tip)):
        if float(pt[2]) < min_z - 1e-9:
            return Verdict(
                False,
                SafetyReason.TABLE_PLANE,
                f"{label} z={float(pt[2]):.4f}m violates table plane clearance {min_z:.4f}m",
            )

    # Check wrist and tip against the base column self-collision cylinder
    for label, pt in (("wrist", wrist), ("tip", tip)):
        r_xy = math.hypot(float(pt[0]), float(pt[1]))
        if r_xy < s.self_collision_radius - 1e-9 and float(pt[2]) < float(model.base_height) + 0.08:
            return Verdict(
                False,
                SafetyReason.SELF_COLLISION,
                f"{label} radial distance {r_xy:.3f}m collides with base column ({s.self_collision_radius:.3f}m)",
            )

    if enforce_workspace:
        if math.hypot(float(tip[0]), float(tip[1])) < s.self_collision_radius - 1e-9:
            return Verdict(
                False,
                SafetyReason.SELF_COLLISION,
                f"tip radial distance {math.hypot(float(tip[0]), float(tip[1])):.3f}m inside self-collision radius",
            )
        cx, cy = s.workspace_centre
        dist_xy = math.hypot(float(tip[0]) - cx, float(tip[1]) - cy)
        if dist_xy > s.workspace_radius + 1e-9:
            return Verdict(
                False,
                SafetyReason.WORKSPACE_BOUNDS,
                f"tip XY ({float(tip[0]):.3f}, {float(tip[1]):.3f}) is {dist_xy:.3f}m from workspace centre, limit {s.workspace_radius:.3f}m",
            )
        if float(tip[2]) < s.z_min - 1e-9 or float(tip[2]) > s.z_max + 1e-9:
            return Verdict(
                False,
                SafetyReason.WORKSPACE_BOUNDS,
                f"tip z={float(tip[2]):.3f}m outside [{s.z_min:.3f}, {s.z_max:.3f}]m",
            )

    return Verdict(True, SafetyReason.OK, "ok")


def preflight_check_plan(
    model: RobotModel,
    poses: Sequence[Mapping[str, float]],
) -> tuple[list[Verdict], int]:
    """Run a pre-flight feasibility check over a nominal sequence of poses (PLAN.md §I).

    Returns ``(verdicts, rejected_infeasible_count)``.
    """
    verdicts = [check_pose(model, p) for p in poses]
    rejected = sum(1 for v in verdicts if not v.approved)
    return verdicts, rejected


class SafetySupervisor:
    """Stateful counter wrapper around the pure :func:`check_pose` function."""

    def __init__(self, model: RobotModel) -> None:
        self.model = model
        self.rejected_infeasible: int = 0
        self.total_checks: int = 0

    def check(
        self,
        pose: Mapping[str, float],
        *,
        opening: float | None = None,
        velocities: Mapping[str, float] | None = None,
        enforce_workspace: bool = True,
    ) -> Verdict:
        self.total_checks += 1
        verdict = check_pose(
            self.model,
            pose,
            opening=opening,
            velocities=velocities,
            enforce_workspace=enforce_workspace,
        )
        if not verdict.approved:
            self.rejected_infeasible += 1
        return verdict
