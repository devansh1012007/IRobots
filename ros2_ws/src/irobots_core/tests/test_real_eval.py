"""Tests for real-frame capture validation and sim-to-real evaluation (PLAN:irobots-v2:C19, C20, T27).

Asserts:
- T27: frame capture validator rejects calibrations with RMS > 5 mm, writes valid
       manifest.jsonl entries, and reports pending status cleanly when data/real/
       has not yet been captured by the operator (never fabricates real frames).
- C20: evaluate_real_frames computes detection rate, false-positive rate on empty
       frames, XY error vs ground-truth corners, and the sim-to-real gap vs
       rendered C02 metrics.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from irobots_core.perception.homography import CameraModel
from irobots_core.perception.real_eval import (
    evaluate_real_dataset,
    validate_and_record_captured_frame,
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


def test_capture_validator_rejects_high_rms_and_records_valid_manifest(
    tmp_path: Path, scene: dict
) -> None:
    """T27: frame capture rejects reprojection RMS > 5 mm and records valid frames to manifest.jsonl."""
    camera = _make_camera(scene)
    board_corners_table = [
        (-0.06, 0.14),
        (0.06, 0.14),
        (0.06, 0.26),
        (-0.06, 0.26),
        (0.0, 0.20),
    ]
    good_corners_px = [camera.project((x, y, 0.0)) for x, y in board_corners_table]
    frame = np.full((camera.height, camera.width, 3), (97, 99, 107), dtype=np.uint8)
    u, v = camera.project((0.0, 0.20, 0.0))
    ui, vi = int(round(u)), int(round(v))
    frame[vi - 28 : vi, ui - 14 : ui + 14] = (220, 50, 50)

    # 1. Valid frame is saved and appended to manifest.jsonl
    ok, entry = validate_and_record_captured_frame(
        out_dir=tmp_path,
        frame_index=0,
        frame_rgb=frame,
        detected_corners_px=good_corners_px,
        board_corners_table_m=board_corners_table,
        ground_truth_objects=[{"label": "red", "xy_m": [0.0, 0.20], "height_m": 0.04, "footprint_m": 0.04}],
        max_rms_m=0.005,
    )
    assert ok is True
    assert entry is not None
    assert entry["reprojection_rms_m"] < 0.005
    assert (tmp_path / "manifest.jsonl").exists()

    # 2. Corrupted corner (>5 mm RMS) is rejected and not written
    bad_corners_px = list(good_corners_px)
    bad_corners_px[4] = (bad_corners_px[4][0] + 45.0, bad_corners_px[4][1] - 40.0)
    ok_bad, entry_bad = validate_and_record_captured_frame(
        out_dir=tmp_path,
        frame_index=1,
        frame_rgb=frame,
        detected_corners_px=bad_corners_px,
        board_corners_table_m=board_corners_table,
        ground_truth_objects=[],
        max_rms_m=0.005,
    )
    assert ok_bad is False
    assert entry_bad is None
    lines = (tmp_path / "manifest.jsonl").read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 1


def test_real_eval_reports_pending_when_data_real_is_absent(tmp_path: Path, scene: dict, robot) -> None:
    """Hard rule §3.3: do NOT fabricate frames; if data/real/ is absent, report pending."""
    missing_dir = tmp_path / "nonexistent_real"
    report = evaluate_real_dataset(
        real_dir=missing_dir,
        scene_cfg=scene,
        workspace_centre=robot.safety.workspace_centre,
        workspace_radius=robot.safety.workspace_radius,
    )
    assert report["status"] == "pending_operator_capture"
    assert report["frames_evaluated"] == 0


def test_real_eval_computes_metrics_and_sim_to_real_gap_on_manifest(
    tmp_path: Path, scene: dict, robot
) -> None:
    """C20: evaluate_real_dataset computes detection rate, XY error, false-positive rate, and gap."""
    camera = _make_camera(scene)
    board_table = [(-0.06, 0.14), (0.06, 0.14), (0.06, 0.26), (-0.06, 0.26)]
    board_px = [camera.project((x, y, 0.0)) for x, y in board_table]

    # Frame 0: cube at (0.0, 0.20) -> front-bottom edge at (0.0, 0.18, 0.0)
    f0 = np.full((camera.height, camera.width, 3), (97, 99, 107), dtype=np.uint8)
    u_fb, v_fb = camera.project((0.0, 0.18, 0.0))
    ui, vi = int(round(u_fb)), int(round(v_fb))
    f0[vi - 28 : vi, ui - 14 : ui + 14] = (220, 50, 50)
    validate_and_record_captured_frame(
        out_dir=tmp_path,
        frame_index=0,
        frame_rgb=f0,
        detected_corners_px=board_px,
        board_corners_table_m=board_table,
        ground_truth_objects=[{"label": "red", "xy_m": [0.0, 0.20], "height_m": 0.04, "footprint_m": 0.04}],
    )

    # Frame 1: empty workspace frame (tests false-positive rate)
    f1 = np.full((camera.height, camera.width, 3), (97, 99, 107), dtype=np.uint8)
    validate_and_record_captured_frame(
        out_dir=tmp_path,
        frame_index=1,
        frame_rgb=f1,
        detected_corners_px=board_px,
        board_corners_table_m=board_table,
        ground_truth_objects=[],
    )

    rendered_ref = {"detection_rate": 1.0, "corrected_err_mm": {"mean": 3.8, "p95": 7.0}}
    report = evaluate_real_dataset(
        real_dir=tmp_path,
        scene_cfg=scene,
        workspace_centre=robot.safety.workspace_centre,
        workspace_radius=robot.safety.workspace_radius,
        rendered_summary=rendered_ref,
    )
    assert report["status"] == "evaluated"
    assert report["frames_evaluated"] == 2
    assert report["detection_rate"] == 1.0
    assert report["false_positive_rate_empty_frames"] == 0.0
    assert report["xy_error_mm"]["p95"] <= 8.0
    assert "sim_to_real_gap" in report
