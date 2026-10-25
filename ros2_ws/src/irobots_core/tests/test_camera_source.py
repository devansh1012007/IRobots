"""Tests for configurable camera source switching (`sim` vs `esp32_cam`) and F3 safety semantics (PLAN:irobots-v2:C65, C67-F3).

Asserts:
- T66: `CameraSourceConfig.from_configs` loads `perception.camera_source` from `config/perception.yaml`, honors `IROBOTS_CAMERA_SOURCE` env override, and raises `ConfigError` on unknown source types, missing `calibration_file` for `esp32_cam`, or missing `BOARD_IP`.
- T67: `SimCameraSource.capture()` renders a `(480, 640, 3)` RGB `uint8` frame from the paused simulator with `timestamp_s == sim.total_steps * sim.dt` (Invariant I6) and `source == CameraSourceMode.SIM`.
- T68: `ESP32CameraSource.capture()` fetches JPEG bytes from a local HTTP `/capture` endpoint, decodes BGR->RGB, resizes native `320x240` frames to the configured `640x480` perception resolution, retries transient failures, and strips credentials from `capture_host`.
- T69 / C67-F3:
  1. `esp32_cam` without a valid `MarkerCalibration` (`calibration_file` missing or RMS > threshold) refuses with `ConfigError` and never falls back to `AnalyticCalibration`.
  2. Hybrid guard: `esp32_cam` + `mujoco` arm backend without `allow_real_camera_with_sim_arm=True` refuses with `ConfigError`; when enabled, labels episode records and `metrics.json` as `"hybrid: real camera + simulated arm"` and sets `task_success_privileged=None` so real frames are never graded with privileged sim ground truth.
  3. Camera timeout / unreachable server returns a typed `CameraCaptureError` that `ClosedLoopRunner` turns into `outcome="camera_unavailable"` (`failure_mode="camera_unavailable"`) while terminating within the I4 deadline.
  4. Both `sim` and `esp32_cam` sources produce the exact same `Detection` schema (`label`, `confidence`, `bbox_px`, `index`, `mask`), and every `episode.jsonl` turn record logs `camera_source` and (for `esp32_cam`) `capture_host` without credentials.
"""

from __future__ import annotations

import http.server
import json
import socketserver
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest
import yaml

from irobots_core.config import ConfigError, load_yaml
from irobots_core.loop import ClosedLoopRunner
from irobots_core.perception.camera_source import (
    CameraCaptureError,
    CameraSourceConfig,
    CameraSourceMode,
    CapturedFrame,
    ESP32CameraSource,
    SimCameraSource,
    build_camera_source,
)
from irobots_core.perception.detector import Detection
from irobots_core.perception.geometric import GeometricDetector
from irobots_core.perception.homography import CameraModel, MarkerCalibration
from irobots_core.robot_model import RobotModel
from irobots_core.sim.mujoco_sim import DeterministicKinematicSim
from irobots_core.sim.scene_builder import build_seeded_scene, load_task_spec


class _LocalESP32CamServer:
    """Thread-safe local HTTP server emulating the ESP32-CAM `GET /capture` JPEG endpoint."""

    def __init__(
        self,
        frame_rgb: np.ndarray,
        *,
        fail_first_n: int = 0,
        always_fail: bool = False,
        delay_s: float = 0.0,
    ) -> None:
        self.frame_rgb = frame_rgb
        self.fail_first_n = fail_first_n
        self.always_fail = always_fail
        self.delay_s = delay_s
        self.requests_seen = 0
        bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        assert ok
        self.jpeg_bytes = bytes(buf)

        parent = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                parent.requests_seen += 1
                if parent.delay_s > 0.0:
                    time.sleep(parent.delay_s)
                if self.path != "/capture":
                    self.send_response(404)
                    self.end_headers()
                    return
                if parent.always_fail or parent.requests_seen <= parent.fail_first_n:
                    self.send_response(503)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(parent.jpeg_bytes)))
                self.end_headers()
                self.wfile.write(parent.jpeg_bytes)

            def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                return

        self._httpd = socketserver.TCPServer(("localhost", 0), Handler)
        self.host, self.port = self._httpd.server_address[0], int(self._httpd.server_address[1])
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)

    @property
    def endpoint(self) -> str:
        return f"{self.host}:{self.port}"

    def __enter__(self) -> _LocalESP32CamServer:
        self._thread.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()
        self._thread.join(timeout=1.0)


def _write_valid_marker_calibration_file(scene: dict[str, Any], path: Path) -> Path:
    cam_cfg = scene["camera"]
    pos = cam_cfg["position"]
    tgt = cam_cfg["target"]
    camera = CameraModel(
        position=(float(pos[0]), float(pos[1]), float(pos[2])),
        target=(float(tgt[0]), float(tgt[1]), float(tgt[2])),
        fov_deg=float(cam_cfg["hfov_deg"]),
        width=int(cam_cfg["width"]),
        height=int(cam_cfg["height"]),
    )
    table_pts = [
        [-0.08, 0.12],
        [0.08, 0.12],
        [0.08, 0.28],
        [-0.08, 0.28],
        [0.0, 0.20],
        [-0.04, 0.16],
    ]
    img_pts = [list(camera.project((x, y, 0.0))) for x, y in table_pts]
    payload = {
        "provenance": "measured",
        "image_points": img_pts,
        "table_points": table_pts,
    }
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    return path


def test_camera_source_config_loads_and_refuses_esp32_without_calibration(
    scene: dict[str, Any], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """T66 / C67-F3: esp32_cam without a valid MarkerCalibration file refuses with ConfigError."""
    monkeypatch.delenv("IROBOTS_CAMERA_SOURCE", raising=False)
    monkeypatch.delenv("IROBOTS_CALIBRATION_FILE", raising=False)
    monkeypatch.delenv("BOARD_IP", raising=False)

    p_cfg = load_yaml("config/perception.yaml")["perception"]
    cfg = CameraSourceConfig.from_configs(p_cfg, scene)
    assert cfg.source is CameraSourceMode.SIM
    assert cfg.width == 640
    assert cfg.height == 480

    # esp32_cam without calibration_file must refuse with ConfigError immediately on load
    monkeypatch.setenv("IROBOTS_CAMERA_SOURCE", "esp32_cam")
    with pytest.raises(ConfigError, match="calibration"):
        CameraSourceConfig.from_configs(p_cfg, scene)

    # esp32_cam with high-RMS (corrupted) calibration file must also refuse with ConfigError
    bad_cal_path = tmp_path / "bad_cal.yaml"
    _write_valid_marker_calibration_file(scene, bad_cal_path)
    bad_data = yaml.safe_load(bad_cal_path.read_text(encoding="utf-8"))
    bad_data["image_points"][0] = [bad_data["image_points"][0][0] + 85.0, bad_data["image_points"][0][1] - 60.0]
    bad_cal_path.write_text(yaml.safe_dump(bad_data), encoding="utf-8")
    with pytest.raises(ConfigError, match="RMS"):
        CameraSourceConfig.from_configs(p_cfg, scene, calibration_file_override=bad_cal_path)

    # esp32_cam with valid calibration file loads MarkerCalibration, then checks BOARD_IP when building source
    valid_cal_path = _write_valid_marker_calibration_file(scene, tmp_path / "valid_cal.yaml")
    cfg_esp = CameraSourceConfig.from_configs(
        p_cfg, scene, calibration_file_override=valid_cal_path
    )
    assert cfg_esp.source is CameraSourceMode.ESP32_CAM
    assert isinstance(cfg_esp.marker_calibration, MarkerCalibration)
    with pytest.raises(ConfigError, match="BOARD_IP"):
        build_camera_source(cfg_esp, sim=None)

    # Unknown camera source mode raises ConfigError
    monkeypatch.setenv("IROBOTS_CAMERA_SOURCE", "webcam_usb")
    with pytest.raises(ConfigError, match="invalid camera source"):
        CameraSourceConfig.from_configs(p_cfg, scene)


def test_sim_camera_source_captures_paused_sim_frame_with_sim_timestamp(
    robot: RobotModel, scene: dict[str, Any]
) -> None:
    """T67 (I6): SimCameraSource renders a (480, 640, 3) RGB frame stamped with sim time without advancing physics."""
    task = load_task_spec("config/tasks/l1.yaml")
    seeded = build_seeded_scene(robot, scene, task, seed=0)
    sim = DeterministicKinematicSim(robot, scene, seeded.to_sim_objects())
    sim.reset(dict(robot.home_pose), robot.gripper.max_opening)
    sim.step(30)

    src = SimCameraSource(sim)
    steps_before = sim.total_steps
    cap1 = src.capture()
    cap2 = src.capture()

    assert sim.total_steps == steps_before, "SimCameraSource must not advance physics during capture (I6)"
    assert isinstance(cap1, CapturedFrame)
    assert cap1.source is CameraSourceMode.SIM
    assert cap1.frame_id == 1
    assert cap2.frame_id == 2
    assert cap1.timestamp_s == pytest.approx(steps_before * sim.dt)
    assert cap1.rgb.shape == (480, 640, 3)
    assert cap1.rgb.dtype == np.uint8
    assert np.array_equal(cap1.rgb, cap2.rgb)


def test_esp32_camera_source_fetches_jpeg_retries_and_strips_credentials_from_host() -> None:
    """T68 / C67-F3: ESP32CameraSource fetches /capture, retries transient errors, resizes 320x240->640x480, and strips credentials."""
    qvga = np.full((240, 320, 3), (97, 99, 107), dtype=np.uint8)
    qvga[100:140, 140:180] = (225, 55, 55)

    with _LocalESP32CamServer(qvga, fail_first_n=1) as srv:
        src = ESP32CameraSource(
            endpoint=f"http://operator:supersecret@{srv.endpoint}",
            capture_path="/capture",
            width=640,
            height=480,
            timeout_s=2.0,
            retries=3,
            resize_to_scene=True,
        )
        assert "supersecret" not in src.capture_host
        assert "operator" not in src.capture_host
        assert src.capture_host == srv.endpoint

        cap = src.capture(sim_time_s=1.25)
        assert isinstance(cap, CapturedFrame)
        assert srv.requests_seen == 2, "expected 1 failed attempt + 1 retry"
        assert cap.source is CameraSourceMode.ESP32_CAM
        assert cap.timestamp_s == pytest.approx(1.25)
        assert cap.rgb.shape == (480, 640, 3)
        centre_rgb = cap.rgb[240, 320]
        assert int(centre_rgb[0]) > 180 and int(centre_rgb[2]) < 90


def test_hybrid_guard_refuses_without_flag_and_labels_hybrid_when_enabled(
    robot: RobotModel, scene: dict[str, Any], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """C67-F3: esp32_cam + mujoco arm backend refuses unless allow_real_camera_with_sim_arm=True, and never grades with privileged truth."""
    cal_file = _write_valid_marker_calibration_file(scene, tmp_path / "marker_cal.yaml")
    task = load_task_spec("config/tasks/l1.yaml")
    seeded = build_seeded_scene(robot, scene, task, seed=3)
    preview_sim = DeterministicKinematicSim(robot, scene, seeded.to_sim_objects())
    preview_sim.reset(dict(robot.home_pose), robot.gripper.max_opening)
    preview_sim.step(50)
    initial_frame = preview_sim.render()
    preview_sim.close()

    with _LocalESP32CamServer(initial_frame) as srv:
        monkeypatch.setenv("IROBOTS_CAMERA_SOURCE", "esp32_cam")
        monkeypatch.setenv("BOARD_IP", f"http://admin:pw123@{srv.endpoint}")

        # 1. Without allow_real_camera_with_sim_arm, ClosedLoopRunner must refuse
        with pytest.raises(ConfigError, match="allow_real_camera_with_sim_arm"):
            ClosedLoopRunner(
                robot,
                scene,
                calibration_file=cal_file,
                allow_real_camera_with_sim_arm=False,
            )

        # 2. With allow_real_camera_with_sim_arm=True, it runs using MarkerCalibration, labels everything hybrid,
        #    sets task_success_privileged=None, and records camera_source + host (no credentials) in episode.jsonl.
        runner_hybrid = ClosedLoopRunner(
            robot,
            scene,
            calibration_file=cal_file,
            allow_real_camera_with_sim_arm=True,
        )
        assert isinstance(runner_hybrid.calibration, MarkerCalibration)
        ep_jsonl = tmp_path / "ep_hybrid.jsonl"
        metrics_json = tmp_path / "metrics_hybrid.json"
        summary = runner_hybrid.run_episode(
            task_path="config/tasks/l1.yaml",
            seed=3,
            jsonl_path=ep_jsonl,
            metrics_path=metrics_json,
        )
        assert summary.camera_source == "esp32_cam"
        assert summary.execution_mode == "hybrid: real camera + simulated arm"
        assert summary.task_success_privileged is None, "Real frames must never be graded with privileged sim ground truth"
        assert summary.task_success_perceived is True

        lines = [json.loads(line) for line in ep_jsonl.read_text(encoding="utf-8").splitlines() if line.strip()]
        turn_lines = [r for r in lines if r.get("type") != "episode_summary"]
        summary_line = next(r for r in lines if r.get("type") == "episode_summary")
        assert len(turn_lines) >= 3
        for rec in turn_lines:
            assert rec["camera_source"] == "esp32_cam"
            assert rec["capture_host"] == srv.endpoint
            assert "pw123" not in json.dumps(rec)
            assert rec["execution_mode"] == "hybrid: real camera + simulated arm"
        assert summary_line["execution_mode"] == "hybrid: real camera + simulated arm"
        assert summary_line["task_success_privileged"] is None

        metrics_payload = json.loads(metrics_json.read_text(encoding="utf-8"))
        assert metrics_payload["execution_mode"] == "hybrid: real camera + simulated arm"
        assert metrics_payload["task_success_privileged"] is None


def test_camera_timeout_produces_camera_unavailable_and_episode_terminates(
    robot: RobotModel, scene: dict[str, Any], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """C67-F3 (I4): dead/timing-out ESP32 camera returns CameraCaptureError -> outcome 'camera_unavailable' and terminates cleanly."""
    cal_file = _write_valid_marker_calibration_file(scene, tmp_path / "marker_cal.yaml")
    blank = np.zeros((240, 320, 3), dtype=np.uint8)

    with _LocalESP32CamServer(blank, always_fail=True, delay_s=0.15) as srv:
        src = ESP32CameraSource(
            endpoint=srv.endpoint,
            timeout_s=0.05,
            retries=3,
        )
        t0 = time.monotonic()
        cap_err = src.capture(sim_time_s=0.1, deadline_s=0.25)
        elapsed = time.monotonic() - t0
        assert isinstance(cap_err, CameraCaptureError)
        assert cap_err.reason == "camera_unavailable"
        assert elapsed < 1.0

        monkeypatch.setenv("IROBOTS_CAMERA_SOURCE", "esp32_cam")
        monkeypatch.setenv("BOARD_IP", srv.endpoint)
        runner = ClosedLoopRunner(
            robot,
            scene,
            calibration_file=cal_file,
            allow_real_camera_with_sim_arm=True,
        )
        ep_jsonl = tmp_path / "ep_dead_cam.jsonl"
        summary = runner.run_episode(
            task_path="config/tasks/l1.yaml",
            seed=1,
            jsonl_path=ep_jsonl,
            camera_source=src,
        )
        assert summary.failure_mode == "camera_unavailable"
        assert summary.task_success_perceived is False
        assert summary.task_success_privileged is None
        records = [json.loads(line) for line in ep_jsonl.read_text(encoding="utf-8").splitlines() if line.strip()]
        turn_recs = [r for r in records if r.get("type") != "episode_summary"]
        assert len(turn_recs) == 1
        assert turn_recs[0]["outcome"] == "camera_unavailable"
        assert turn_recs[0]["camera_source"] == "esp32_cam"
        assert turn_recs[0]["capture_host"] == srv.endpoint


def test_both_camera_sources_produce_identical_detection_schema(
    robot: RobotModel, scene: dict[str, Any]
) -> None:
    """C67-F3: SimCameraSource and ESP32CameraSource frames produce the exact same Detection schema."""
    task = load_task_spec("config/tasks/l1.yaml")
    seeded = build_seeded_scene(robot, scene, task, seed=2)
    sim = DeterministicKinematicSim(robot, scene, seeded.to_sim_objects())
    sim.reset(dict(robot.home_pose), robot.gripper.max_opening)
    sim.step(50)

    sim_src = SimCameraSource(sim)
    sim_cap = sim_src.capture()
    assert isinstance(sim_cap, CapturedFrame)

    cam_cfg = scene["camera"]
    pos = cam_cfg["position"]
    tgt = cam_cfg["target"]
    camera = CameraModel(
        position=(float(pos[0]), float(pos[1]), float(pos[2])),
        target=(float(tgt[0]), float(tgt[1]), float(tgt[2])),
        fov_deg=float(cam_cfg["hfov_deg"]),
        width=int(cam_cfg["width"]),
        height=int(cam_cfg["height"]),
    )
    detector = GeometricDetector(
        camera=camera,
        workspace_centre=robot.safety.workspace_centre,
        workspace_radius=robot.safety.workspace_radius,
    )

    with _LocalESP32CamServer(sim_cap.rgb) as srv:
        esp_src = ESP32CameraSource(endpoint=srv.endpoint, width=640, height=480)
        esp_cap = esp_src.capture(sim_time_s=sim_cap.timestamp_s)
        assert isinstance(esp_cap, CapturedFrame)

    dets_sim = detector.detect(sim_cap.rgb)
    dets_esp = detector.detect(esp_cap.rgb)
    assert len(dets_sim) == 1
    assert len(dets_esp) == 1
    for d in (dets_sim[0], dets_esp[0]):
        assert isinstance(d, Detection)
        assert isinstance(d.label, str)
        assert isinstance(d.confidence, float)
        assert isinstance(d.bbox_px, tuple) and len(d.bbox_px) == 4
        assert isinstance(d.index, int)
        assert isinstance(d.mask, np.ndarray)
    assert dets_sim[0].label == dets_esp[0].label
    sim.close()
