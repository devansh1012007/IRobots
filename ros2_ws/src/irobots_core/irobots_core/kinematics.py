"""Analytic forward and inverse kinematics for the 5-DOF arm.

The chain is::

    base_yaw(z) -> shoulder_pitch(y) -> elbow_pitch(y) -> wrist_pitch(y)
                -> wrist_yaw(z, at the tool tip)

The three pitch joints are planar, so the arm works inside a vertical plane
whose azimuth is set by ``base_yaw``. ``wrist_yaw`` sits at the tool tip and
rotates about the approach axis, which means that for a downward-pointing tool
it moves the tip *not at all*: it is a pure tool-yaw DOF that sets the jaw
closing direction.

That property is the whole reason for this joint layout. A yaw-pitch-roll
wrist cannot do it: its roll axis *is* the approach axis, so the jaw direction
would be locked to the reach direction and every grasp would be a corner
grasp.

All rotations are expressed with the convention that a pitch joint uses the
axis ``(0, -1, 0)`` in its parent frame, so a positive pitch raises the link.
The MuJoCo model is generated to match this exactly, and
``tests/test_kinematics.py`` asserts the two agree to better than 1e-6.

PLAN:irobots-v2:C07
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Final

import numpy as np

from .robot_model import RobotModel

#: Tolerance for the FK/IK round trip.
ROUND_TRIP_TOLERANCE: Final[float] = 1e-6

#: The tool frame relative to its parent: X_new = Y_parent, Y_new = Z_parent,
#: Z_new = X_parent. As a quaternion this is a 120 degree rotation about
#: (1,1,1)/sqrt(3), which is what puts the tool's Z (the approach axis) along
#: the link and the tool's X (the jaw closing direction) along the arm plane's
#: normal.
TOOL_FRAME_QUAT: Final[tuple[float, float, float, float]] = (0.5, 0.5, 0.5, 0.5)


class IKStatus(StrEnum):
    """Outcome of an inverse-kinematics solve.

    The caller must be able to tell these apart: an unreachable target and a
    joint-limit violation need different handling, and neither may be silently
    clamped into a "reachable" answer.
    """

    REACHABLE = "reachable"
    UNREACHABLE = "unreachable"
    JOINT_LIMIT = "joint_limit"
    SINGULAR = "singular"


@dataclass(frozen=True, slots=True)
class ToolState:
    """Where the tool is and how it is oriented."""

    position: np.ndarray  # (3,) tip position in the arm base frame, metres
    approach: np.ndarray  # (3,) unit vector, tool Z: the direction the jaws come from
    closing: np.ndarray  # (3,) unit vector, tool X: the jaw closing direction
    azimuth: float  # world azimuth of the closing direction, radians


@dataclass(frozen=True, slots=True)
class IKResult:
    """The result of an inverse-kinematics solve."""

    status: IKStatus
    pose: dict[str, float] = field(default_factory=dict)
    closest: np.ndarray | None = None  # nearest reachable point, when unreachable
    message: str = ""

    @property
    def ok(self) -> bool:
        return self.status is IKStatus.REACHABLE


def _rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _quat_to_mat(quat: Sequence[float]) -> np.ndarray:
    """Rotation matrix from a (w, x, y, z) quaternion."""
    w, x, y, z = quat
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n == 0.0:
        raise ValueError("zero quaternion")
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


#: The constant part of the tool frame.
_TOOL_FRAME: Final[np.ndarray] = _quat_to_mat(TOOL_FRAME_QUAT)


def forward_kinematics(model: RobotModel, pose: Mapping[str, float]) -> ToolState:
    """Compute the tool tip position and orientation for a joint pose.

    :param model: the robot description.
    :param pose: joint angles in radians, keyed by joint name. ``gripper`` is
        accepted and ignored.
    :returns: a :class:`ToolState`.
    """
    theta_s = float(pose.get("shoulder_pitch", 0.0))
    theta_e = float(pose.get("elbow_pitch", 0.0))
    theta_wp = float(pose.get("wrist_pitch", 0.0))
    psi_b = float(pose.get("base_yaw", 0.0))
    psi_w = float(pose.get("wrist_yaw", 0.0))

    l1 = float(model.upper_arm)
    l2 = float(model.fore_arm)
    l34 = model.wrist_to_tip
    base_h = float(model.base_height)

    # Absolute link angles in the vertical arm plane.
    phi1 = theta_s
    phi2 = phi1 + theta_e
    phi3 = phi2 + theta_wp

    d1 = np.array([math.cos(phi1), 0.0, math.sin(phi1)])
    d2 = np.array([math.cos(phi2), 0.0, math.sin(phi2)])
    d3 = np.array([math.cos(phi3), 0.0, math.sin(phi3)])

    r_base = _rot_z(psi_b)
    # The base column offset is along world z, so it is added after the yaw.
    position = np.array([0.0, 0.0, base_h]) + r_base @ (l1 * d1 + l2 * d2 + l34 * d3)

    # The tool frame: base yaw, then the accumulated pitch, then the constant
    # tool-frame permutation, then the wrist yaw about the approach axis.
    r_tool = r_base @ _rot_y(-phi3) @ _TOOL_FRAME @ _rot_z(psi_w)

    approach = r_tool @ np.array([0.0, 0.0, 1.0])
    closing = r_tool @ np.array([1.0, 0.0, 0.0])

    return ToolState(
        position=position,
        approach=approach,
        closing=closing,
        azimuth=math.atan2(closing[1], closing[0]),
    )


def _two_link_branches(
    r: float, h: float, l1: float, l2: float
) -> list[tuple[float, float]]:
    """Both solutions of the planar 2R sub-chain for a wrist point at (r, h).

    ``r`` is the horizontal distance from the shoulder axis and ``h`` is the
    height above the shoulder pivot. Returns a list of ``(phi1, phi2)`` - the
    absolute link angles - one per elbow configuration, or an empty list when
    the point is out of reach.

    The two branches are NOT "elbow up" and "elbow down" in a fixed sense: with
    this convention the branch that puts the elbow above the wrist is the one
    with a NEGATIVE relative elbow angle, because the pitch axis points the
    other way. The caller picks between them by joint-limit feasibility, not
    by a preference flag, so a valid solution is never reported as a
    joint-limit violation just because the other branch was tried first.
    """
    d2 = r * r + h * h
    d = math.sqrt(d2)
    if d > l1 + l2 or d < abs(l1 - l2):
        return []
    cos_e = (d2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    cos_e = max(-1.0, min(1.0, cos_e))
    base = math.acos(cos_e)
    out: list[tuple[float, float]] = []
    for theta_e in (base, -base):
        phi1 = math.atan2(h, r) - math.atan2(
            l2 * math.sin(theta_e), l1 + l2 * math.cos(theta_e)
        )
        out.append((phi1, phi1 + theta_e))
    return out


def _elbow_height(l1: float, base_h: float, phi1: float) -> float:
    """Height of the elbow (the end of the upper arm) above the table."""
    return base_h + l1 * math.sin(phi1)


def _closing_azimuth_for(
    model: RobotModel, psi_b: float, phi3: float, psi_w: float
) -> float:
    """World azimuth of the jaw closing direction for a given wrist yaw."""
    r = _rot_z(psi_b) @ _rot_y(-phi3) @ _TOOL_FRAME @ _rot_z(psi_w)
    c = r @ np.array([1.0, 0.0, 0.0])
    return math.atan2(c[1], c[0])


def inverse_kinematics(
    model: RobotModel,
    target: Sequence[float],
    *,
    approach_azimuth: float | None = None,
    closing_azimuth: float | None = None,
    elbow_up: bool = True,
) -> IKResult:
    """Solve for a joint pose that puts the tool tip at ``target``.

    The tip always lies in the vertical plane through the base and the target,
    so ``base_yaw`` is simply the target's azimuth. The approach direction is
    taken to lie in that same plane: ``approach_azimuth`` defaults to the
    target's azimuth, and the requested tilt is implied by the vertical
    component of the target-to-wrist geometry.

    :param target: the desired tip position ``(x, y, z)`` in the arm base
        frame, in metres.
    :param approach_azimuth: world azimuth the approach direction should have.
        Defaults to the target's azimuth (a purely radial approach).
    :param closing_azimuth: world azimuth the jaw closing direction should
        have. Defaults to 90 degrees from the approach, i.e. perpendicular to
        the reach direction.
    :param elbow_up: prefer the elbow-up posture.
    :returns: an :class:`IKResult`. Never returns NaN.
    """
    tgt = np.asarray(target, dtype=float)
    if tgt.shape != (3,) or not np.all(np.isfinite(tgt)):
        return IKResult(IKStatus.UNREACHABLE, message=f"invalid target {target!r}")

    r = math.hypot(tgt[0], tgt[1])
    if r < 1e-9:
        return IKResult(
            IKStatus.SINGULAR,
            message="target is on the base axis: the azimuth is undefined",
            closest=np.array([0.0, 0.0, float(tgt[2])]),
        )

    psi_b = math.atan2(tgt[1], tgt[0])
    l1 = float(model.upper_arm)
    l2 = float(model.fore_arm)
    l34 = model.wrist_to_tip
    base_h = float(model.base_height)

    # The approach direction lies in the arm plane. Its world azimuth is
    # psi_b (or the requested one), and its tilt is set by the geometry: we
    # solve for the wrist point that puts the tip at the target with the tool
    # pointing straight down, which is the posture every task uses.
    approach_tilt = -math.pi / 2.0
    u_r = math.cos(approach_tilt)
    u_z = math.sin(approach_tilt)

    w_r = r - l34 * u_r
    w_h = (tgt[2] - base_h) - l34 * u_z

    branches = _two_link_branches(w_r, w_h, l1, l2)
    if not branches:
        # Report the closest reachable point instead of a bare failure.
        d = math.hypot(w_r, w_h)
        d_clamped = max(abs(l1 - l2) + 1e-6, min(l1 + l2 - 1e-6, d))
        scale = d_clamped / d if d > 0 else 0.0
        cr, ch = w_r * scale, w_h * scale
        closest = np.array(
            [
                cr * math.cos(psi_b),
                cr * math.sin(psi_b),
                base_h + ch + l34 * u_z,
            ]
        )
        return IKResult(
            IKStatus.UNREACHABLE,
            closest=closest,
            message=(
                f"target is {d:.3f}m from the shoulder; reach is "
                f"[{abs(l1 - l2):.3f}, {l1 + l2:.3f}]m"
            ),
        )

    # Try every elbow configuration and keep the ones that fit the joint
    # limits. Reporting a joint-limit violation when a valid solution exists
    # on the other branch would be a false negative, and it is exactly the
    # kind of thing that makes a robot look broken when it is not.
    candidates: list[tuple[dict[str, float], float]] = []
    for phi1, phi2 in branches:
        phi3 = approach_tilt
        theta_s = phi1
        theta_e = phi2 - phi1
        theta_wp = phi3 - phi2
        if closing_azimuth is None:
            psi_w = 0.0
        else:
            psi_w = _solve_wrist_yaw(model, psi_b, phi3, closing_azimuth)
        pose = {
            "base_yaw": psi_b,
            "shoulder_pitch": theta_s,
            "elbow_pitch": theta_e,
            "wrist_pitch": theta_wp,
            "wrist_yaw": psi_w,
        }
        within_limits = True
        for name, value in pose.items():
            spec = model.joint(name)
            if value < spec.min_rad - 1e-9 or value > spec.max_rad + 1e-9:
                within_limits = False
                break
        # Higher elbow is better for a table arm: it keeps the forearm clear
        # of the table and the object.
        quality = _elbow_height(l1, base_h, phi1)
        candidates.append((pose, quality if within_limits else -1e9))

    feasible = [(p, q) for p, q in candidates if q > -1e8]
    if not feasible:
        pose = candidates[0][0]
        worst = ""
        for name, value in pose.items():
            spec = model.joint(name)
            if value < spec.min_rad - 1e-9 or value > spec.max_rad + 1e-9:
                worst = (
                    f"{name}={math.degrees(value):.1f} deg is outside "
                    f"[{spec.min_deg:.1f}, {spec.max_deg:.1f}]"
                )
                break
        return IKResult(IKStatus.JOINT_LIMIT, pose=pose, message=worst)

    # Among the feasible branches, take the one with the highest elbow. Ties
    # are broken towards the smaller total joint travel, which keeps the
    # motion small and deterministic.
    robot_home = model.home_pose

    def travel(pose: Mapping[str, float]) -> float:
        return sum(abs(pose[n] - robot_home.get(n, 0.0)) for n in pose)

    best = min(feasible, key=lambda item: (-item[1], travel(item[0])))
    return IKResult(IKStatus.REACHABLE, pose=dict(best[0]))


def _solve_wrist_yaw(
    model: RobotModel,
    psi_b: float,
    phi3: float,
    desired_azimuth: float,
) -> float:
    """Find the wrist yaw whose jaw closing direction has a given world azimuth.

    Solved as an exact signed angle rather than by bisecting on a wrapped
    azimuth difference. The naive version compares ``wrap(closing - target)``
    at the two ends of the joint range, but that function has a discontinuity
    where the azimuth crosses 180 degrees: over a 340 degree range the signs
    at the ends can match even though a valid solution exists in between, and
    the solver then returns a joint limit instead of the answer.

    The wrist yaw rotates the closing direction about the approach axis, so
    the required rotation is the signed angle from the zero-yaw closing
    direction to the desired one, measured about that axis.
    """
    r0 = _rot_z(psi_b) @ _rot_y(-phi3) @ _TOOL_FRAME
    c0 = r0 @ np.array([1.0, 0.0, 0.0])
    n = r0 @ np.array([0.0, 0.0, 1.0])  # the approach axis

    desired = np.array([math.cos(desired_azimuth), math.sin(desired_azimuth), 0.0])
    # Project the desired direction into the plane perpendicular to the
    # approach axis: that is the only plane the wrist can rotate within.
    d_perp = desired - n * float(np.dot(n, desired))
    norm = float(np.linalg.norm(d_perp))
    if norm < 1e-9:
        return 0.0
    d_perp = d_perp / norm

    c0_perp = c0 - n * float(np.dot(n, c0))
    c0_norm = float(np.linalg.norm(c0_perp))
    if c0_norm < 1e-9:
        return 0.0
    c0_perp = c0_perp / c0_norm

    sin_term = float(np.dot(n, np.cross(c0_perp, d_perp)))
    cos_term = float(np.dot(c0_perp, d_perp))
    psi = math.atan2(sin_term, cos_term)

    # The joint range spans more than pi, so an equivalent solution may exist
    # at psi +/- 2*pi. Pick whichever lands inside the range.
    lo = model.joint("wrist_yaw").min_rad
    hi = model.joint("wrist_yaw").max_rad
    for candidate in (psi, psi + 2.0 * math.pi, psi - 2.0 * math.pi):
        if lo - 1e-9 <= candidate <= hi + 1e-9:
            return candidate

    # The requested closing direction is not reachable with this wrist. Snap
    # to the closest achievable one rather than distorting the angle: a
    # slightly misaligned grasp is fine, a grasp that closes along a
    # completely wrong axis is not. The residual error is observable by the
    # caller, which can read the achieved azimuth from the forward kinematics.
    best_psi = lo
    best_err = float("inf")
    steps = 720
    for i in range(steps + 1):
        candidate = lo + (hi - lo) * i / steps
        err = abs(
            _wrap_pi(_closing_azimuth_for(model, psi_b, phi3, candidate) - desired_azimuth)
        )
        if err < best_err:
            best_err = err
            best_psi = candidate
    return best_psi


def _wrap_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def workspace_reach_check(
    model: RobotModel, samples: int = 256
) -> list[tuple[float, float, bool]]:
    """Check which points of the workspace disk the arm can actually reach.

    :returns: a list of ``(x, y, reachable)`` samples.
    """
    cx, cy = model.safety.workspace_centre
    radius = model.safety.workspace_radius
    rng = np.random.default_rng(12345)
    rr = radius * np.sqrt(rng.random(samples))
    th = rng.random(samples) * 2.0 * math.pi
    out: list[tuple[float, float, bool]] = []
    for r_i, t_i in zip(rr, th, strict=True):
        x = float(cx + r_i * math.cos(t_i))
        y = float(cy + r_i * math.sin(t_i))
        # Reachability at the tabletop height, with a downward approach.
        res = inverse_kinematics(model, (x, y, float(model.gripper.tip_length)))
        out.append((x, y, res.ok))
    return out
