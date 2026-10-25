"""Tests for grasp candidate synthesis (PLAN:irobots-v2:C16, T23, T24).

Asserts:
- T23: candidate's required jaw opening >= object width (with clearance margin) and <= gripper.max_opening.
- T24: candidate's XY equals the D/2-corrected contact point on the table plane.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from irobots_core.perception.detector import Detection
from irobots_core.perception.grasp import GraspCandidate, propose_grasp_candidate
from irobots_core.perception.height import HeightEstimate, HeightSource
from irobots_core.perception.homography import (
    AnalyticCalibration,
    CameraModel,
    contact_correction,
)


def _make_camera(scene: dict) -> CameraModel:
    cam_cfg = scene["camera"]
    return CameraModel(
        position=(float(cam_cfg["position"][0]), float(cam_cfg["position"][1]), float(cam_cfg["position"][2])),
        target=(float(cam_cfg["target"][0]), float(cam_cfg["target"][1]), float(cam_cfg["target"][2])),
        fov_deg=float(cam_cfg["hfov_deg"]),
        width=int(cam_cfg["width"]),
        height=int(cam_cfg["height"]),
    )


def test_grasp_candidate_opening_exceeds_object_width_and_respects_max(robot, scene: dict) -> None:
    """T23: candidate's required opening >= object width and <= robot.gripper.max_opening."""
    camera = _make_camera(scene)
    cal = AnalyticCalibration(camera=camera)
    u, v = camera.project((0.02, 0.18, 0.0))
    det = Detection(
        label="red block",
        confidence=0.95,
        bbox_px=(u - 18.0, v - 35.0, u + 18.0, v),
        index=0,
    )
    h_est = HeightEstimate(value=0.040, source=HeightSource.PROBE, confidence=0.95)
    cand = propose_grasp_candidate(
        detection=det,
        calibration=cal,
        camera_xy=(camera.position[0], camera.position[1]),
        height=h_est,
        object_width_m=0.040,
        footprint_m=0.040,
        max_gripper_opening_m=robot.gripper.max_opening,
    )
    assert isinstance(cand, GraspCandidate)
    assert cand.required_opening_m >= 0.040
    assert cand.required_opening_m <= robot.gripper.max_opening
    assert cand.grasp_z_m == pytest.approx(0.020)


def test_grasp_candidate_xy_equals_depth_corrected_contact_point(robot, scene: dict) -> None:
    """T24: candidate's XY equals the D/2-corrected contact point."""
    camera = _make_camera(scene)
    cal = AnalyticCalibration(camera=camera)
    u, v = camera.project((-0.03, 0.19, 0.0))
    mask = np.zeros((camera.height, camera.width), dtype=bool)
    ui, vi = int(round(u)), int(round(v))
    mask[vi - 24 : vi, ui - 16 : ui + 16] = True

    det = Detection(
        label="blue block",
        confidence=0.98,
        bbox_px=(u - 16.0, v - 24.0, u + 16.0, v),
        index=0,
        mask=mask,
    )
    h_est = HeightEstimate(value=0.040, source=HeightSource.PRIOR, confidence=0.70)
    cand = propose_grasp_candidate(
        detection=det,
        calibration=cal,
        camera_xy=(camera.position[0], camera.position[1]),
        height=h_est,
        object_width_m=0.040,
        footprint_m=0.040,
        max_gripper_opening_m=robot.gripper.max_opening,
    )

    bu, bv = det.bottom_center_px
    raw_xy = cal.to_arm_base_xy(bu, bv)
    dx, dy = contact_correction((camera.position[0], camera.position[1]), raw_xy, 0.040)
    expected_xy = (raw_xy[0] + dx, raw_xy[1] + dy)
    assert cand.contact_xy_m == pytest.approx(expected_xy, abs=1e-9)
    assert math.isfinite(cand.jaw_yaw_rad)
