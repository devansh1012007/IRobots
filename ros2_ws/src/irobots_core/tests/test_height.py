"""Tests for monocular height estimation (PLAN:irobots-v2:C15, T20, T21, T22, D.3).

Asserts:
- T20: Tier P probe recovers object height H within 5 mm on a synthetic cube.
- T21: Tier C class prior path returns its configured table value from config/perception.yaml.
- T22: Tier V vanishing-point silhouette path reports its ~+72% bias on a 4 cm cube at 35 deg
       and is disabled by default.
"""

from __future__ import annotations

import pytest

from irobots_core.config import load_yaml
from irobots_core.perception.height import (
    HeightEstimator,
    HeightSource,
    estimate_height_prior,
    estimate_height_probe,
    estimate_height_vanishing_point,
)
from irobots_core.perception.homography import CameraModel


def _make_camera(scene: dict) -> CameraModel:
    cam_cfg = scene["camera"]
    return CameraModel(
        position=(float(cam_cfg["position"][0]), float(cam_cfg["position"][1]), float(cam_cfg["position"][2])),
        target=(float(cam_cfg["target"][0]), float(cam_cfg["target"][1]), float(cam_cfg["target"][2])),
        fov_deg=float(cam_cfg["hfov_deg"]),
        width=int(cam_cfg["width"]),
        height=int(cam_cfg["height"]),
    )


def test_probe_tier_recovers_height_within_5mm(scene: dict) -> None:
    """T20: descending jaw occlusion probe recovers a 40 mm cube's height within 5 mm."""
    camera = _make_camera(scene)
    true_h = 0.040
    xy = (0.0, 0.20)
    # Top-front edge of the 40 mm cube projects to row v_top
    _u_top, v_top = camera.project((xy[0], xy[1] - 0.02, true_h))

    # Simulate descending probe steps from z=0.068 down to 0.028 in 4 mm steps
    probe_samples: list[tuple[float, float]] = []
    for z_jaw in (0.068, 0.064, 0.060, 0.056, 0.052, 0.048, 0.044, 0.040, 0.036):
        _u_j, v_jaw = camera.project((xy[0], xy[1] - 0.02, z_jaw))
        probe_samples.append((z_jaw, v_jaw))

    est = estimate_height_probe(
        probe_samples=probe_samples,
        object_top_row_px=v_top,
        occlusion_row_tolerance_px=3.0,
    )
    assert est.source is HeightSource.PROBE
    assert abs(est.value - true_h) <= 0.005, f"probe error {abs(est.value - true_h)*1000:.2f} mm > 5 mm"
    assert est.confidence >= 0.8


def test_prior_tier_returns_configured_table_value() -> None:
    """T21: class prior path returns its table value from config/perception.yaml."""
    p_cfg = load_yaml("config/perception.yaml")["perception"]
    est_red = estimate_height_prior("red block", p_cfg["height_priors"])
    assert est_red.source is HeightSource.PRIOR
    assert est_red.value == pytest.approx(0.040)
    assert est_red.provenance == "design"

    est_bin = estimate_height_prior("bin", p_cfg["height_priors"])
    assert est_bin.source is HeightSource.PRIOR
    assert est_bin.value == pytest.approx(0.030)


def test_vanishing_point_tier_reports_bias_and_is_disabled_by_default(scene: dict) -> None:
    """T22: VP silhouette path over-estimates a 4 cm cube at 35 deg by ~+70% and is disabled by default."""
    camera = _make_camera(scene)
    p_cfg = load_yaml("config/perception.yaml")["perception"]
    estimator = HeightEstimator.from_config(p_cfg, camera=camera)
    assert estimator.enable_vanishing_point is False

    # Calling VP through the estimator while disabled must raise ValueError unless forced for RQ2 ablation
    with pytest.raises(ValueError, match="disabled"):
        estimator.estimate_vanishing_point(bbox_height_px=47.3, object_xy=(0.0, 0.20))

    # Direct measurement for RQ2 ablation: 4 cm cube at (0, 0.20), camera tilted 35 deg
    _u_bot, v_bot = camera.project((0.0, 0.18, 0.0))
    _u_top, v_top = camera.project((0.0, 0.22, 0.04))
    h_px = abs(v_bot - v_top)
    vp_est = estimate_height_vanishing_point(
        camera=camera,
        bbox_height_px=h_px,
        object_xy=(0.0, 0.20),
        tilt_deg=35.0,
    )
    assert vp_est.source is HeightSource.VANISHING_POINT
    # Silhouette equation confounds H*cos(35) + D*sin(35), yielding ~0.068 m (+70% over 0.040 m)
    assert vp_est.value > 0.060
    assert vp_est.bias_note != ""
