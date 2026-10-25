"""RQ2 experiment: Tier P (probe) vs Tier C (class prior) vs Tier V (vanishing point) height estimation (PLAN:irobots-v2:C45, D.3, SC2).

Primary metric: ``abs(H_hat - H)`` and XY error on a frozen 200-frame synthetic
set mixing standard 40 mm cubes and tall 90 mm blocks (where the 40 mm class
prior is wrong by 50 mm and Tier V silhouette inversion is confounded by
footprint depth D).
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from irobots_core.config import REPO_ROOT, load_yaml
from irobots_core.perception.geometric import GeometricDetector
from irobots_core.perception.height import (
    estimate_height_prior,
    estimate_height_probe,
    estimate_height_vanishing_point,
)
from irobots_core.perception.homography import (
    AnalyticCalibration,
    CameraModel,
    contact_correction,
)
from irobots_core.robot_model import load_robot
from irobots_core.sim.mujoco_sim import DeterministicKinematicSim


def run_rq2(*, n_frames: int = 200, out_path: str | Path | None = None) -> dict[str, Any]:
    robot = load_robot()
    scene_cfg = load_yaml("config/scene.yaml")["scene"]
    p_cfg = load_yaml("config/perception.yaml")["perception"]
    cam_cfg = scene_cfg["camera"]
    camera = CameraModel(
        position=(float(cam_cfg["position"][0]), float(cam_cfg["position"][1]), float(cam_cfg["position"][2])),
        target=(float(cam_cfg["target"][0]), float(cam_cfg["target"][1]), float(cam_cfg["target"][2])),
        fov_deg=float(cam_cfg["hfov_deg"]),
        width=int(cam_cfg["width"]),
        height=int(cam_cfg["height"]),
    )
    analytic = AnalyticCalibration(camera=camera)
    detector = GeometricDetector(
        camera=camera,
        workspace_centre=robot.safety.workspace_centre,
        workspace_radius=robot.safety.workspace_radius,
    )

    rng = np.random.default_rng(2026)
    cx, cy = robot.safety.workspace_centre
    radius = robot.safety.workspace_radius * 0.82

    detected_count = 0
    xy_errors_mm: list[float] = []
    err_probe_mm: list[float] = []
    err_prior_mm: list[float] = []
    err_vp_mm: list[float] = []

    for i in range(n_frames):
        r = radius * math.sqrt(float(rng.random()))
        theta = float(rng.random()) * 2.0 * math.pi
        tx = cx + r * math.cos(theta)
        ty = cy + r * math.sin(theta)
        # Half the frames are 40 mm cubes, half are 90 mm tall blocks (both red)
        is_tall = (i % 2) == 1
        true_h = 0.090 if is_tall else 0.040
        footprint = 0.040

        sim = DeterministicKinematicSim(
            robot,
            scene_cfg,
            [
                {
                    "name": "target_block",
                    "size": [footprint, footprint, true_h],
                    "pos": [tx, ty, true_h / 2.0],
                    "rgba": "0.85 0.25 0.25 1",
                }
            ],
        )
        frame = sim.render()
        dets = detector.detect(frame)
        sim.close()

        if dets:
            detected_count += 1
            d = dets[0]
            bu, bv = d.bottom_center_px
            raw_xy = analytic.to_arm_base_xy(bu, bv)
            dx, dy = contact_correction((camera.position[0], camera.position[1]), raw_xy, footprint)
            corr_xy = (raw_xy[0] + dx, raw_xy[1] + dy)
            xy_errors_mm.append(math.hypot(corr_xy[0] - tx, corr_xy[1] - ty) * 1000.0)

        # Tier P (probe): descends in 4 mm steps and detects occlusion row of top-front edge
        _u_top, v_top = camera.project((tx, ty - footprint / 2.0, true_h))
        probe_samples = [
            (z_jaw, camera.project((tx, ty - footprint / 2.0, z_jaw))[1])
            for z_jaw in np.arange(0.104, 0.016, -0.004)
        ]
        h_probe = estimate_height_probe(probe_samples, v_top, occlusion_row_tolerance_px=3.0)
        err_probe_mm.append(abs(h_probe.value - true_h) * 1000.0)

        # Tier C (class prior): assumes "red block" is 40 mm
        h_prior = estimate_height_prior("red block", p_cfg["height_priors"])
        err_prior_mm.append(abs(h_prior.value - true_h) * 1000.0)

        # Tier V (vanishing point / silhouette inversion): confounded by D*sin(theta)
        _u_b, v_bot = camera.project((tx, ty - footprint / 2.0, 0.0))
        _u_t, v_back_top = camera.project((tx, ty + footprint / 2.0, true_h))
        h_vp = estimate_height_vanishing_point(
            camera,
            bbox_height_px=abs(v_bot - v_back_top),
            object_xy=(tx, ty),
            tilt_deg=35.0,
        )
        err_vp_mm.append(abs(h_vp.value - true_h) * 1000.0)

    det_rate = detected_count / n_frames
    p95_xy = float(np.percentile(xy_errors_mm, 95)) if xy_errors_mm else float("nan")

    report: dict[str, Any] = {
        "rq": "RQ2",
        "sim_backend": "fallback sim",
        "question": "How much does monocular height uncertainty cost, and does verified probing recover it?",
        "frozen_frames": n_frames,
        "sc2_frozen_200_frame_set": {
            "detection_rate": det_rate,
            "mean_xy_error_mm": float(np.mean(xy_errors_mm)),
            "p95_xy_error_mm": p95_xy,
            "gate": {
                "criterion": "SC2",
                "detection_rate_required": 0.95,
                "p95_xy_error_mm_required": 8.0,
                "passed": bool(det_rate >= 0.95 and p95_xy <= 8.0),
            },
        },
        "height_tiers_abs_error_mm": {
            "tier_p_probe": {
                "mean_mm": float(np.mean(err_probe_mm)),
                "p95_mm": float(np.percentile(err_probe_mm, 95)),
                "extra_steps_cost": 1,
            },
            "tier_c_class_prior": {
                "mean_mm": float(np.mean(err_prior_mm)),
                "p95_mm": float(np.percentile(err_prior_mm, 95)),
                "extra_steps_cost": 0,
            },
            "tier_v_vanishing_point_disabled": {
                "mean_mm": float(np.mean(err_vp_mm)),
                "p95_mm": float(np.percentile(err_vp_mm, 95)),
                "extra_steps_cost": 0,
                "note": "Confounded by footprint depth D*sin(theta); disabled by default",
            },
        },
    }
    target = Path(out_path) if out_path else REPO_ROOT / "runs" / "phase6" / "rq2.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    raise SystemExit(0 if run_rq2() else 1)
