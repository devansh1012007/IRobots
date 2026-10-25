"""Tests for homography, camera projection, and contact correction (PLAN:irobots-v2:C02, C13).

Tests T15, T16, T17 plus camera frame orientation and D/2 contact correction.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from irobots_core.perception.homography import (
    AnalyticCalibration,
    CameraModel,
    MarkerCalibration,
    contact_correction,
)
from irobots_core.sim.mujoco_model import look_at_frame


def _make_camera(scene: dict) -> CameraModel:
    cam_cfg = scene["camera"]
    return CameraModel(
        position=tuple(float(c) for c in cam_cfg["position"]),  # type: ignore[arg-type]
        target=tuple(float(c) for c in cam_cfg["target"]),  # type: ignore[arg-type]
        fov_deg=float(cam_cfg["hfov_deg"]),
        width=int(cam_cfg["width"]),
        height=int(cam_cfg["height"]),
    )


def test_camera_frame_is_right_side_up(scene: dict) -> None:
    """C02 / C13: camera +X is right (+world X) and image row v increases downward (-camera Y)."""
    camera = _make_camera(scene)
    rot = camera.rotation
    assert rot[0, 0] > 0.99, f"camera X axis is inverted: {rot[:, 0]}"
    assert rot[2, 1] > 0.0, f"camera Y axis points down instead of up: {rot[:, 1]}"

    xyaxes, _ = look_at_frame(camera.position, camera.target)
    parts = [float(x) for x in xyaxes.split()]
    np.testing.assert_allclose(parts[:3], rot[:, 0], atol=1e-5)
    np.testing.assert_allclose(parts[3:6], rot[:, 1], atol=1e-5)

    _u_far, v_far = camera.project((0.0, 0.25, 0.0))
    _u_near, v_near = camera.project((0.0, 0.15, 0.0))
    assert v_near > v_far, f"near point should have larger row v than far point: {v_near} <= {v_far}"


def test_analytic_calibration_recovers_known_xy_exactly(scene: dict) -> None:
    """T15: analytic calibration recovers known table-plane XY to floating-point precision."""
    camera = _make_camera(scene)
    cal = AnalyticCalibration(camera=camera, table_z=0.0)
    rng = np.random.default_rng(42)
    for _ in range(50):
        x = float(rng.uniform(-0.08, 0.08))
        y = float(rng.uniform(0.12, 0.28))
        u, v = camera.project((x, y, 0.0))
        rx, ry = cal.to_arm_base_xy(u, v)
        assert math.hypot(rx - x, ry - y) < 1e-9


def test_marker_calibration_matches_analytic_on_synthetic_frames(scene: dict) -> None:
    """T16: marker calibration error vs analytic on synthetic points is well within 8 mm."""
    camera = _make_camera(scene)
    analytic = AnalyticCalibration(camera=camera, table_z=0.0)
    marker_table = [
        (-0.08, 0.12),
        (0.08, 0.12),
        (0.08, 0.28),
        (-0.08, 0.28),
        (0.0, 0.20),
        (-0.04, 0.16),
        (0.04, 0.24),
    ]
    marker_img = [camera.project((x, y, 0.0)) for x, y in marker_table]
    marker_cal = MarkerCalibration.fit(marker_img, marker_table)
    assert marker_cal.reprojection_rms_m is not None
    assert marker_cal.reprojection_rms_m < 1e-6

    rng = np.random.default_rng(99)
    for _ in range(40):
        x = float(rng.uniform(-0.08, 0.08))
        y = float(rng.uniform(0.12, 0.28))
        u, v = camera.project((x, y, 0.0))
        ax, ay = analytic.to_arm_base_xy(u, v)
        mx, my = marker_cal.to_arm_base_xy(u, v)
        assert math.hypot(mx - ax, my - ay) < 0.001


def test_marker_calibration_refuses_high_rms(scene: dict) -> None:
    """T17: MarkerCalibration.fit refuses when reprojection RMS exceeds threshold."""
    camera = _make_camera(scene)
    marker_table = [
        (-0.08, 0.12),
        (0.08, 0.12),
        (0.08, 0.28),
        (-0.08, 0.28),
        (0.0, 0.20),
    ]
    marker_img = [camera.project((x, y, 0.0)) for x, y in marker_table]
    corrupted_img = list(marker_img)
    corrupted_img[4] = (corrupted_img[4][0] + 40.0, corrupted_img[4][1] - 35.0)
    with pytest.raises(ValueError, match="exceeds"):
        MarkerCalibration.fit(corrupted_img, marker_table, max_rms_m=0.005)


def test_contact_correction_removes_front_edge_bias(scene: dict) -> None:
    """D.3 / C02: back-projecting the front-bottom edge + contact_correction lands within 8 mm."""
    camera = _make_camera(scene)
    analytic = AnalyticCalibration(camera=camera, table_z=0.0)
    footprint = 0.04
    cx, cy = camera.position[0], camera.position[1]

    rng = np.random.default_rng(7)
    errors_mm = []
    for _ in range(50):
        tx = float(rng.uniform(-0.08, 0.08))
        ty = float(rng.uniform(0.12, 0.28))
        u_fb, v_fb = camera.project((tx, ty - footprint / 2.0, 0.0))
        raw = analytic.to_arm_base_xy(u_fb, v_fb)
        dx, dy = contact_correction((cx, cy), raw, footprint)
        corr = (raw[0] + dx, raw[1] + dy)
        errors_mm.append(math.hypot(corr[0] - tx, corr[1] - ty) * 1000.0)
    assert float(np.percentile(errors_mm, 95)) <= 8.0
