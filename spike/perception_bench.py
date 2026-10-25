#!/usr/bin/env python
"""C02 - the perception gate (PLAN:irobots-v2:C02).

**What this measures:** can the system find a 4 cm cube in the camera's view,
and where is it? The two numbers the plan requires are the detection rate and
the XY localisation error.

**What it deliberately does not do:** use the simulator's ground truth for
anything the system would have to perceive. The cube's true position is used
only to score the answer.

Three things are compared, because the difference between them is the point:

1. **The raw bottom-edge back-projection.** This is what a naive
   implementation does, and it is biased: the bottom edge of a box resting on
   a table is its FRONT bottom edge, so it lands D/2 toward the camera.
2. **The D/2-corrected back-projection.** The correction is applied using the
   class prior's footprint size.
3. **The marker calibration.** A four-point homography fitted the way real
   hardware would fit one, compared against the exact analytic calibration, to
   measure what the manual calibration path costs.

Usage::

    python spike/perception_bench.py --trials 50
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / "irobots_core"))

import numpy as np  # noqa: E402

from irobots_core.config import load_yaml  # noqa: E402
from irobots_core.perception.geometric import GeometricDetector  # noqa: E402
from irobots_core.perception.homography import (  # noqa: E402
    AnalyticCalibration,
    CameraModel,
    MarkerCalibration,
    contact_correction,
)
from irobots_core.robot_model import load_robot  # noqa: E402

CUBE_NAME = "cube"
CUBE_SIZE = 0.04
#: The class prior for the cube's footprint, in metres.
FOOTPRINT_M = 0.04


def sample_pose(
    rng: np.random.Generator, centre: tuple[float, float], radius: float
) -> tuple[float, float, float]:
    """Sample the exact same (x, y, yaw) sequence as C01's grasp_bench.py."""
    r = radius * math.sqrt(float(rng.random()))
    theta = float(rng.random()) * 2.0 * math.pi
    x = centre[0] + r * math.cos(theta)
    y = centre[1] + r * math.sin(theta)
    yaw = float(rng.random()) * math.pi
    return x, y, yaw


def _rasterize_synthetic_cube_frame(
    camera: CameraModel,
    scene: dict[str, Any],
    cube_xy: tuple[float, float],
    cube_size: float = CUBE_SIZE,
) -> np.ndarray:
    """Pinhole-exact software rasteriser fallback when MuJoCo GL is absent."""
    import cv2

    frame = np.full((camera.height, camera.width, 3), (38, 46, 64), dtype=np.uint8)
    table = scene["table"]
    tx, ty = (float(c) for c in table.get("position", [0.0, 0.12]))
    sx, sy = (float(c) for c in table["size_xy"])
    table_corners = [
        (tx - sx / 2.0, ty - sy / 2.0, 0.0),
        (tx + sx / 2.0, ty - sy / 2.0, 0.0),
        (tx + sx / 2.0, ty + sy / 2.0, 0.0),
        (tx - sx / 2.0, ty + sy / 2.0, 0.0),
    ]
    t_poly = np.array(
        [[int(round(u)), int(round(v))] for u, v in (camera.project(p) for p in table_corners)],
        dtype=np.int32,
    )
    cv2.fillPoly(frame, [t_poly], (97, 99, 107))

    cx, cy = cube_xy
    h = cube_size / 2.0
    # Draw front face and top face of the axis-aligned cube
    front_face = [
        (cx - h, cy - h, 0.0),
        (cx + h, cy - h, 0.0),
        (cx + h, cy - h, cube_size),
        (cx - h, cy - h, cube_size),
    ]
    top_face = [
        (cx - h, cy - h, cube_size),
        (cx + h, cy - h, cube_size),
        (cx + h, cy + h, cube_size),
        (cx - h, cy + h, cube_size),
    ]
    for face, rgb in ((top_face, (225, 70, 70)), (front_face, (205, 58, 58))):
        poly = np.array(
            [[int(round(u)), int(round(v))] for u, v in (camera.project(p) for p in face)],
            dtype=np.int32,
        )
        cv2.fillPoly(frame, [poly], rgb)
    return frame


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trials", type=int, default=50)
    ap.add_argument("--seed", type=int, default=500)
    ap.add_argument("--conf", type=float, default=0.30)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--vocabulary", type=str, default="cube,block,red block,blue block")
    ap.add_argument("--no-semantic", action="store_true", help="skip the learned baseline")
    ap.add_argument("--out", type=str, default=str(REPO_ROOT / "runs" / "spike" / "perception"))
    args = ap.parse_args()

    robot = load_robot()
    scene = load_yaml("config/scene.yaml")["scene"]
    cam_cfg = scene["camera"]
    camera = CameraModel(
        position=tuple(float(c) for c in cam_cfg["position"]),  # type: ignore[arg-type]
        target=tuple(float(c) for c in cam_cfg["target"]),  # type: ignore[arg-type]
        fov_deg=float(cam_cfg["hfov_deg"]),
        width=int(cam_cfg["width"]),
        height=int(cam_cfg["height"]),
    )
    analytic = AnalyticCalibration(camera=camera)

    objects = [
        {
            "name": CUBE_NAME,
            "size": [CUBE_SIZE] * 3,
            "pos": [0.0, 0.2, CUBE_SIZE],
            "mass": 0.04,
            "friction": 0.6,
            "rgba": "0.85 0.25 0.25 1",
        }
    ]
    sim = None
    render_backend = "mujoco"
    try:
        from irobots_core.sim.mujoco_sim import MuJoCoSim

        sim = MuJoCoSim(robot, scene, objects)
    except Exception:
        render_backend = "pinhole_rasterizer"

    wc = robot.safety.workspace_centre
    wr = robot.safety.workspace_radius
    marker_table = [
        (wc[0] - wr * 0.8, wc[1] - wr * 0.8),
        (wc[0] + wr * 0.8, wc[1] - wr * 0.8),
        (wc[0] + wr * 0.8, wc[1] + wr * 0.8),
        (wc[0] - wr * 0.8, wc[1] + wr * 0.8),
    ]
    marker_image = [camera.project((p[0], p[1], 0.0)) for p in marker_table]
    marker_cal = MarkerCalibration.fit(marker_image, marker_table)
    print(
        f"marker calibration: RMS = {marker_cal.reprojection_rms_m * 1000:.3f} mm "
        f"(fitted from 4 points)"
    )

    geometric = GeometricDetector(
        camera=camera,
        workspace_centre=robot.safety.workspace_centre,
        workspace_radius=robot.safety.workspace_radius,
    )
    semantic = None
    if not args.no_semantic:
        try:
            from irobots_core.perception.detector import WorldDetector

            print("loading the learned detector (baseline)...")
            semantic = WorldDetector(imgsz=args.imgsz, conf_threshold=args.conf)
            semantic.set_vocabulary(
                [v.strip() for v in args.vocabulary.split(",") if v.strip()]
            )
        except Exception as exc:
            print(f"semantic baseline skipped ({type(exc).__name__}: {exc})")

    rng = np.random.default_rng(args.seed)
    centre = robot.safety.workspace_centre
    radius = robot.safety.workspace_radius * 0.85

    records: list[dict[str, Any]] = []
    detect_times: list[float] = []
    render_times: list[float] = []

    for i in range(args.trials):
        x, y, _yaw = sample_pose(rng, centre, radius)
        if sim is not None:
            sim.reset(dict(robot.home_pose), robot.gripper.max_opening)
            sim.set_object_pose(CUBE_NAME, (x, y, CUBE_SIZE / 2.0), [1.0, 0.0, 0.0, 0.0])
            sim.step(120)
            truth = sim.object_position(CUBE_NAME)
            t0 = time.perf_counter()
            frame = sim.render()
            render_times.append(time.perf_counter() - t0)
        else:
            truth = np.array([x, y, CUBE_SIZE / 2.0], dtype=float)
            t0 = time.perf_counter()
            frame = _rasterize_synthetic_cube_frame(camera, scene, (x, y), CUBE_SIZE)
            render_times.append(time.perf_counter() - t0)

        t0 = time.perf_counter()
        detections = geometric.detect(frame)
        dt = time.perf_counter() - t0

        semantic_hits: list[str] = []
        semantic_ms = 0.0
        if semantic is not None:
            t1 = time.perf_counter()
            semantic_hits = [d.label for d in semantic.detect(frame)]
            semantic_ms = (time.perf_counter() - t1) * 1000
        detect_times.append(dt)

        truth_px = camera.project((float(truth[0]), float(truth[1]), CUBE_SIZE / 2.0))
        best = None
        best_dist = float("inf")
        for d in detections:
            cx_px, cy_px = d.center_px
            dist = math.hypot(cx_px - truth_px[0], cy_px - truth_px[1])
            if dist < best_dist:
                best_dist = dist
                best = d

        rec: dict[str, Any] = {
            "trial": i,
            "truth_xy": [float(truth[0]), float(truth[1])],
            "truth_px": [truth_px[0], truth_px[1]],
            "n_detections": len(detections),
            "labels": [d.label for d in detections],
            "detect_s": dt,
            "semantic_labels": semantic_hits,
            "semantic_ms": semantic_ms,
            "render_s": render_times[-1],
        }

        if best is not None and best_dist < 40.0:
            bu, bv = best.bottom_center_px
            raw = analytic.to_arm_base_xy(bu, bv)
            dx, dy = contact_correction(
                (camera.position[0], camera.position[1]), (raw[0], raw[1]), FOOTPRINT_M
            )
            corrected = (raw[0] + dx, raw[1] + dy)
            marker_raw = marker_cal.to_arm_base_xy(bu, bv)
            mdx, mdy = contact_correction(
                (camera.position[0], camera.position[1]),
                (marker_raw[0], marker_raw[1]),
                FOOTPRINT_M,
            )
            marker = (marker_raw[0] + mdx, marker_raw[1] + mdy)
            rec.update(
                {
                    "detected": True,
                    "label": best.label,
                    "confidence": best.confidence,
                    "raw_xy": list(raw),
                    "corrected_xy": list(corrected),
                    "marker_xy": list(marker),
                    "raw_err_mm": math.dist(raw, (float(truth[0]), float(truth[1]))) * 1000,
                    "corrected_err_mm": math.dist(
                        corrected, (float(truth[0]), float(truth[1]))
                    )
                    * 1000,
                    "marker_err_mm": math.dist(marker, (float(truth[0]), float(truth[1])))
                    * 1000,
                }
            )
        else:
            rec.update(
                {
                    "detected": False,
                    "raw_err_mm": None,
                    "corrected_err_mm": None,
                    "marker_err_mm": None,
                }
            )
        records.append(rec)
        if (i + 1) % 10 == 0:
            n_ok = sum(1 for r in records if r["detected"])
            print(f"  {i + 1}/{args.trials} frames, {n_ok} with the cube")

    if sim is not None:
        sim.close()

    detected = [r for r in records if r["detected"]]
    rate = len(detected) / len(records) if records else 0.0

    def p95(values: list[float]) -> float:
        return float(np.percentile(values, 95)) if values else float("nan")

    raw_errs = [r["raw_err_mm"] for r in detected]
    corr_errs = [r["corrected_err_mm"] for r in detected]
    marker_errs = [r["marker_err_mm"] for r in detected]

    summary = {
        "trials": len(records),
        "render_backend": render_backend,
        "detection_rate": rate,
        "mean_detect_s": float(np.mean(detect_times)) if detect_times else 0.0,
        "p95_detect_s": p95(detect_times),
        "mean_render_s": float(np.mean(render_times)) if render_times else 0.0,
        "raw_err_mm": {
            "mean": float(np.mean(raw_errs)) if raw_errs else None,
            "p95": p95(raw_errs),
        },
        "corrected_err_mm": {
            "mean": float(np.mean(corr_errs)) if corr_errs else None,
            "p95": p95(corr_errs),
        },
        "marker_err_mm": {
            "mean": float(np.mean(marker_errs)) if marker_errs else None,
            "p95": p95(marker_errs),
        },
        "marker_calibration_rms_mm": (
            marker_cal.reprojection_rms_m * 1000 if marker_cal.reprojection_rms_m else None
        ),
        "gate": {
            "detection_rate_required": 0.95,
            "p95_xy_error_mm_required": 8.0,
        },
    }
    summary["gate"]["passed"] = bool(
        rate >= 0.95 and corr_errs and p95(corr_errs) <= 8.0
    )

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "perception_bench.json").write_text(
        json.dumps({"summary": summary, "records": records}, indent=2), encoding="utf-8"
    )

    print()
    print(f"render backend        : {render_backend}")
    print(f"detection rate        : {rate:.1%}  (gate >= 95%)")
    print(
        f"XY error, raw         : mean {np.mean(raw_errs) if raw_errs else 0:.1f} mm, "
        f"p95 {p95(raw_errs):.1f} mm"
    )
    print(
        f"XY error, D/2-corrected: mean {np.mean(corr_errs) if corr_errs else 0:.1f} mm, "
        f"p95 {p95(corr_errs):.1f} mm  (gate <= 8 mm)"
    )
    print(
        f"XY error, marker calib: mean {np.mean(marker_errs) if marker_errs else 0:.1f} mm, "
        f"p95 {p95(marker_errs):.1f} mm"
    )
    print(
        f"detect latency        : mean {summary['mean_detect_s'] * 1000:.0f} ms, "
        f"p95 {summary['p95_detect_s'] * 1000:.0f} ms"
    )
    if semantic is not None:
        sem_hit = sum(1 for r in records if r["semantic_labels"])
        sem_ms = [r["semantic_ms"] for r in records]
        print(
            f"learned baseline      : {sem_hit}/{len(records)} frames with ANY detection, "
            f"mean {np.mean(sem_ms):.0f} ms"
        )
    print(f"render latency        : mean {summary['mean_render_s'] * 1000:.0f} ms")
    print(f"GATE {'PASS' if summary['gate']['passed'] else 'FAIL'}")
    return 0 if summary["gate"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
