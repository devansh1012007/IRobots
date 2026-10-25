"""Grasp candidate synthesis from perceived XY, height, and mask principal axis.

Deriving the jaw closing yaw from the mask's principal axis (rather than
locking it to the base reach direction) is what prevents corner grasps
(PLAN:irobots-v2:C16).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .detector import Detection
from .height import HeightEstimate
from .homography import Calibration, contact_correction

#: Default radial clearance added to each finger pad when pre-opening the jaws.
DEFAULT_OPENING_MARGIN_M: float = 0.016


@dataclass(frozen=True, slots=True)
class GraspCandidate:
    """One synthesized grasp pose for a detected object."""

    raw_xy_m: tuple[float, float]
    contact_xy_m: tuple[float, float]
    grasp_z_m: float
    hover_z_m: float
    jaw_yaw_rad: float
    required_opening_m: float
    object_width_m: float
    height_estimate: HeightEstimate


def principal_axis_yaw_rad(mask: np.ndarray | None) -> float:
    """Estimate the horizontal principal axis orientation from a binary object mask."""
    if mask is None:
        return 0.0
    ys, xs = np.where(mask)
    if xs.size < 4:
        return 0.0
    pts = np.column_stack([xs.astype(float), -ys.astype(float)])
    pts -= pts.mean(axis=0, keepdims=True)
    cov = (pts.T @ pts) / max(1, pts.shape[0] - 1)
    eigvals, eigvecs = np.linalg.eigh(cov)
    major = eigvecs[:, int(np.argmax(eigvals))]
    # Closing axis is perpendicular to the major elongation axis (or aligned with the cube face)
    yaw = math.atan2(float(major[1]), float(major[0]))
    # Fold into [-pi/4, pi/4] for 4-fold symmetric blocks so wrist motion stays minimal
    while yaw > math.pi / 4.0:
        yaw -= math.pi / 2.0
    while yaw < -math.pi / 4.0:
        yaw += math.pi / 2.0
    return yaw


def propose_grasp_candidate(
    detection: Detection,
    calibration: Calibration,
    camera_xy: tuple[float, float],
    height: HeightEstimate,
    *,
    object_width_m: float = 0.040,
    footprint_m: float = 0.040,
    max_gripper_opening_m: float = 0.070,
    apply_depth_correction: bool = True,
    hover_clearance_m: float = 0.040,
) -> GraspCandidate:
    """Synthesize a :class:`GraspCandidate` from a detection, calibration, and height estimate."""
    if object_width_m > max_gripper_opening_m:
        raise ValueError(
            f"object width {object_width_m:.3f}m exceeds max gripper opening {max_gripper_opening_m:.3f}m"
        )

    bu, bv = detection.bottom_center_px
    raw_xy = calibration.to_arm_base_xy(bu, bv)
    if apply_depth_correction:
        dx, dy = contact_correction(camera_xy, raw_xy, footprint_m)
        contact_xy = (raw_xy[0] + dx, raw_xy[1] + dy)
    else:
        contact_xy = raw_xy

    grasp_z = max(0.006, 0.5 * float(height.value))
    hover_z = float(height.value) + hover_clearance_m
    jaw_yaw = principal_axis_yaw_rad(detection.mask)
    required_opening = min(max_gripper_opening_m, max(object_width_m, object_width_m + DEFAULT_OPENING_MARGIN_M))

    return GraspCandidate(
        raw_xy_m=raw_xy,
        contact_xy_m=contact_xy,
        grasp_z_m=grasp_z,
        hover_z_m=hover_z,
        jaw_yaw_rad=jaw_yaw,
        required_opening_m=required_opening,
        object_width_m=object_width_m,
        height_estimate=height,
    )
