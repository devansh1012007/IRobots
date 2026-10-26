#!/usr/bin/env python
"""ESP32-CAM real-frame capture and evaluation CLI (PLAN:irobots-v2:C19, C20, C67-F8).

Follows the specification in ``docs/hardware/CAMERA_CAPTURE.md``:
- Capture mode: ``python tools/capture_real_frames.py --board board.yaml --out data/real/ --count 300``
- Eval mode:    ``python tools/capture_real_frames.py --eval data/real/ --report runs/real/eval.json``

Never fabricates frames. When the camera endpoint is unreachable or ``BOARD_IP``
is not configured in the environment, it reports ``pending_operator_capture``
cleanly without inventing data.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
for _pkg in ("irobots_bringup", "irobots_eval", "irobots_msgs", "irobots_nodes", "irobots_core"):
    _src = REPO_ROOT / "ros2_ws" / "src" / _pkg
    if _src.is_dir() and str(_src) not in sys.path:
        sys.path.insert(0, str(_src))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from irobots_core.config import env_or, load_yaml  # noqa: E402
from irobots_core.perception.camera_source import ESP32CameraSource  # noqa: E402
from irobots_core.perception.real_eval import (  # noqa: E402
    evaluate_real_dataset,
    validate_and_record_captured_frame,
)
from irobots_core.robot_model import load_robot  # noqa: E402


def fetch_frame_rgb(capture_url: str, timeout_s: float = 5.0) -> np.ndarray:
    """Fetch a single JPEG frame from the ESP32-CAM ``/capture`` endpoint."""
    with urllib.request.urlopen(capture_url, timeout=timeout_s) as resp:  # noqa: S310
        raw = resp.read()
    arr = np.frombuffer(raw, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("failed to decode JPEG frame from camera")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def detect_charuco_or_chessboard_corners(
    frame_rgb: np.ndarray,
    expected_count: int,
) -> list[tuple[float, float]] | None:
    """Detect calibration corners in ``frame_rgb`` using OpenCV."""
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    found, corners = cv2.findChessboardCorners(gray, (5, 4), None)
    if not found or corners is None or len(corners) < expected_count:
        return None
    pts = corners.reshape(-1, 2)[:expected_count]
    return [(float(p[0]), float(p[1])) for p in pts]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--board", type=str, default=None, help="path to board.yaml with corner coordinates")
    ap.add_argument("--out", type=str, default=str(REPO_ROOT / "data" / "real"))
    ap.add_argument("--count", type=int, default=300)
    ap.add_argument("--eval", dest="eval_dir", type=str, default=None, help="evaluate existing data/real/")
    ap.add_argument(
        "--report",
        type=str,
        default=str(REPO_ROOT / "runs" / "real" / "eval.json"),
    )
    ap.add_argument(
        "--strict",
        action="store_true",
        help="exit non-zero if BOARD_IP is unset during capture mode",
    )
    args = ap.parse_args()

    robot = load_robot()
    scene = load_yaml("config/scene.yaml")["scene"]

    if args.eval_dir is not None:
        rendered_path = REPO_ROOT / "runs" / "spike" / "perception" / "perception_bench.json"
        rendered_summary: dict[str, Any] | None = None
        if rendered_path.exists():
            rendered_summary = json.loads(rendered_path.read_text(encoding="utf-8")).get("summary")
        report = evaluate_real_dataset(
            real_dir=args.eval_dir,
            scene_cfg=scene,
            workspace_centre=robot.safety.workspace_centre,
            workspace_radius=robot.safety.workspace_radius,
            rendered_summary=rendered_summary,
        )
        rep_path = Path(args.report)
        rep_path.parent.mkdir(parents=True, exist_ok=True)
        rep_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))
        return 0

    board_ip = env_or("BOARD_IP")
    if not board_ip or not args.board:
        msg = (
            "pending_operator_capture: BOARD_IP is not set in environment (or --board omitted); "
            "skipping live ESP32-CAM capture. Set BOARD_IP=<esp32-cam-ip> when hardware is connected."
        )
        print(msg, file=sys.stderr)
        return 2 if args.strict else 0

    board_path = Path(args.board)
    if not board_path.exists() and not (REPO_ROOT / args.board).exists():
        print(f"capture board file not found: {args.board}", file=sys.stderr)
        return 2 if args.strict else 0

    board_cfg = load_yaml(args.board)
    board_corners = [(float(p[0]), float(p[1])) for p in board_cfg["corners_table_m"]]
    cam_source = ESP32CameraSource(endpoint=board_ip, capture_path="/capture")
    out_dir = Path(args.out)

    accepted = 0
    attempts = 0
    while accepted < args.count and attempts < args.count * 3:
        attempts += 1
        try:
            frame = cam_source.capture().rgb
        except Exception as exc:
            print(f"capture retry ({exc})", file=sys.stderr)
            time.sleep(0.5)
            continue
        corners_px = detect_charuco_or_chessboard_corners(frame, len(board_corners))
        if corners_px is None:
            continue
        ok, _ = validate_and_record_captured_frame(
            out_dir=out_dir,
            frame_index=accepted,
            frame_rgb=frame,
            detected_corners_px=corners_px,
            board_corners_table_m=board_corners,
            ground_truth_objects=board_cfg.get("objects", []),
        )
        if ok:
            accepted += 1
    print(f"captured {accepted}/{args.count} valid frames into {out_dir}")
    return 0 if accepted >= args.count else 1


if __name__ == "__main__":
    raise SystemExit(main())
