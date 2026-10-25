"""Real-frame capture validation and sim-to-real perception evaluation (PLAN:irobots-v2:C19, C20).

Never fabricates frames. When ``data/real/manifest.jsonl`` is absent,
:func:`evaluate_real_dataset` returns ``status="pending_operator_capture"`` so
``docs/RESULTS.md`` can report the real-frame claim honestly as pending
operator hardware capture.
"""

from __future__ import annotations

import json
import math
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .geometric import GeometricDetector
from .homography import CameraModel, MarkerCalibration, contact_correction


def validate_and_record_captured_frame(
    out_dir: Path,
    frame_index: int,
    frame_rgb: np.ndarray,
    detected_corners_px: Sequence[tuple[float, float]],
    board_corners_table_m: Sequence[tuple[float, float]],
    ground_truth_objects: Sequence[Mapping[str, Any]] = (),
    *,
    max_rms_m: float = 0.005,
    timestamp_s: float | None = None,
) -> tuple[bool, dict[str, Any] | None]:
    """Validate ChArUco/marker reprojection RMS <= 5 mm and write frame + manifest entry (C19).

    A frame whose reprojection RMS exceeds ``max_rms_m`` is rejected, not kept
    (``docs/hardware/CAMERA_CAPTURE.md`` §4).
    """
    try:
        cal = MarkerCalibration.fit(
            detected_corners_px,
            board_corners_table_m,
            max_rms_m=max_rms_m,
            min_points=4,
        )
    except ValueError:
        return (False, None)

    out_dir.mkdir(parents=True, exist_ok=True)
    fname = f"frame_{frame_index:04d}.png"
    fpath = out_dir / fname
    bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(fpath), bgr)

    rms = float(cal.reprojection_rms_m or 0.0)
    entry: dict[str, Any] = {
        "file": fname,
        "frame_index": frame_index,
        "timestamp": float(timestamp_s if timestamp_s is not None else time.time()),
        "detected_corners_px": [[float(u), float(v)] for u, v in detected_corners_px],
        "board_corners_table_m": [[float(x), float(y)] for x, y in board_corners_table_m],
        "reprojection_rms_m": rms,
        "homography": cal.homography.tolist(),
        "objects": [dict(obj) for obj in ground_truth_objects],
    }
    manifest_path = out_dir / "manifest.jsonl"
    with manifest_path.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(entry) + "\n")
    return (True, entry)


def evaluate_real_dataset(
    real_dir: str | Path,
    scene_cfg: Mapping[str, Any],
    workspace_centre: tuple[float, float],
    workspace_radius: float,
    *,
    rendered_summary: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Evaluate detector and marker calibration on captured real frames (C20)."""
    rdir = Path(real_dir)
    manifest_path = rdir / "manifest.jsonl"
    if not manifest_path.exists():
        return {
            "status": "pending_operator_capture",
            "frames_evaluated": 0,
            "note": (
                "data/real/manifest.jsonl is absent; awaiting operator ESP32-CAM "
                "capture per docs/hardware/CAMERA_CAPTURE.md"
            ),
        }

    cam_cfg = scene_cfg["camera"]
    camera = CameraModel(
        position=tuple(float(c) for c in cam_cfg["position"]),  # type: ignore[arg-type]
        target=tuple(float(c) for c in cam_cfg["target"]),  # type: ignore[arg-type]
        fov_deg=float(cam_cfg["hfov_deg"]),
        width=int(cam_cfg["width"]),
        height=int(cam_cfg["height"]),
    )
    detector = GeometricDetector(
        camera=camera,
        workspace_centre=workspace_centre,
        workspace_radius=workspace_radius,
    )

    lines = [
        line.strip()
        for line in manifest_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not lines:
        return {
            "status": "pending_operator_capture",
            "frames_evaluated": 0,
            "note": "manifest.jsonl is empty",
        }

    total_gt_objects = 0
    detected_gt_objects = 0
    empty_frames = 0
    empty_frames_with_fp = 0
    xy_errors_mm: list[float] = []

    for line in lines:
        rec = json.loads(line)
        img_path = rdir / str(rec["file"])
        bgr = cv2.imread(str(img_path))
        if bgr is None:
            continue
        frame_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        corners_px = [(float(p[0]), float(p[1])) for p in rec["detected_corners_px"]]
        corners_tb = [(float(p[0]), float(p[1])) for p in rec["board_corners_table_m"]]
        cal = MarkerCalibration.fit(corners_px, corners_tb)

        dets = detector.detect(frame_rgb)
        gt_objs = rec.get("objects", [])
        if not gt_objs:
            empty_frames += 1
            if len(dets) > 0:
                empty_frames_with_fp += 1
            continue

        for gt in gt_objs:
            total_gt_objects += 1
            gx, gy = float(gt["xy_m"][0]), float(gt["xy_m"][1])
            footprint = float(gt.get("footprint_m", 0.04))
            best_err = float("inf")
            for d in dets:
                bu, bv = d.bottom_center_px
                raw_xy = cal.to_arm_base_xy(bu, bv)
                dx, dy = contact_correction(
                    (camera.position[0], camera.position[1]), raw_xy, footprint
                )
                corr_xy = (raw_xy[0] + dx, raw_xy[1] + dy)
                err_mm = math.hypot(corr_xy[0] - gx, corr_xy[1] - gy) * 1000.0
                if err_mm < best_err:
                    best_err = err_mm
            if best_err < 40.0:
                detected_gt_objects += 1
                xy_errors_mm.append(best_err)

    det_rate = detected_gt_objects / total_gt_objects if total_gt_objects > 0 else 0.0
    fp_rate = empty_frames_with_fp / empty_frames if empty_frames > 0 else 0.0
    mean_xy = float(np.mean(xy_errors_mm)) if xy_errors_mm else None
    p95_xy = float(np.percentile(xy_errors_mm, 95)) if xy_errors_mm else None

    result: dict[str, Any] = {
        "status": "evaluated",
        "frames_evaluated": len(lines),
        "detection_rate": det_rate,
        "false_positive_rate_empty_frames": fp_rate,
        "xy_error_mm": {"mean": mean_xy, "p95": p95_xy},
        "gate": {
            "detection_rate_required": 0.95,
            "p95_xy_error_mm_required": 8.0,
            "passed": bool(det_rate >= 0.95 and p95_xy is not None and p95_xy <= 8.0),
        },
    }
    if rendered_summary is not None:
        rend_det = float(rendered_summary.get("detection_rate", 0.0))
        rend_p95 = rendered_summary.get("corrected_err_mm", {}).get("p95")
        result["sim_to_real_gap"] = {
            "detection_rate_delta": det_rate - rend_det,
            "p95_xy_error_mm_delta": (
                (p95_xy - float(rend_p95)) if (p95_xy is not None and rend_p95 is not None) else None
            ),
        }
    return result
