"""Tests for configurable camera source switching (`sim` vs `esp32_cam`) (PLAN:irobots-v2:C65, T66-T70).

Asserts:
- T66: `CameraSourceConfig.from_Configs` loads `perception.camera_source` from `config/perception.yaml`, honors `IROBOTS_CAMERA_SOURCE` env override, and raises `ConfigError` on unknown source types or missing `BOARD_IP` when `esp32_cam` is selected.
- T67: `SimCameraSource.capture()` renders a `(480, 640, 3)` RGB `uint8` frame from the paused simulator with `timestamp_s == sim.total_steps * sim.dt` (Invariant I6) and `source == CameraSourceMode.SIM`.
- T68: `ESP32CameraSource.capture()` fetches JPEG bytes from a local HTTP `/capture` endpoint, decodes BGR->RGB, resizes native `320x240` frames to the configured `640x480` perception resolution, and retries transient failures.
- T69: `ClosedLoopRunner` operates end-to-end on both `source="sim"` and `source="esp32_cam"` (with the local HTTP camera server serving the scene's JPEG frames), completing the L1 pick-and-lift task with identical downstream perception, planning, safety, and verification code.
- T70: `test_no_endpoint_is_hardcoded_in_the_core` (T12) and `test_the_core_does_not_import_ros` remain green.
"""

from __future__ import annotations

import http.server
import socketserver
import threading
from pathlib import Path

import cv2
import numpy as np
import pytest

from irobots_core.config import ConfigError, load_yaml
from irobots_core.loop import ClosedLoopRunner
from irobots_core.perception.camera_source import (
    CameraSourceConfig,
    CameraSourceMode,
    ESP32CameraSource,
    SimCameraSource,
    build_camera_source,
)
from irobots_core.sim.mujoco_sim import DeterministicKinematicSim
from irobots_core.sim.scene_builder import build_seeded_scene, load_task_spec


class _LocalESP32CamServer:
    """Thread-safe local HTTP server emulating the ESP32-CAM `GET /capture` JPEG endpoint."""

    def __init__(self, frame_rgb: np.ndarray, *, fail_first_n: int = 0) -> None:
        self.frame_rgb = frame_rgb
        self.fail_first_n = fail_first_n
        self.requests_seen = 0
        bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        assert ok
        self.jpeg_bytes = bytes(buf)

        parent = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                parent.requests_seen += 1
                if self.path != "/capture":
                    self.send_response(404)
                    self.end_headers()
                    return
                if parent.requests_seen <= parent.fail_first_n:
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

    def update_frame(self, frame_rgb: np.ndarray) -> None:
        bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        assert ok
        self.jpeg_bytes = bytes(buf)

    def __enter__(self) -> _LocalESP32CamServer:
        self._thread.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()
        self._thread.join(timeout=1.0)


def test_camera_source_config_loads_and_validates_modes(
    scene: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    """T66: config loader parses sim/esp32_cam, respects env overrides, and rejects invalid sources or missing BOARD_IP."""
    monkeypatch.delenv("IROBOTS_CAMERA_SOURCE", raising=False)
    monkeypatch.delenv("BOARD_IP", raising=False)

    p_cfg = load_yaml("config/perception.yaml")["perception"]
    cfg = CameraSourceConfig.from_configs(p_cfg, scene)
    assert cfg.source is CameraSourceMode.SIM
    assert cfg.width == 640
    assert cfg.height == 480

    # Env override to esp32_cam without BOARD_IP must fail loudly when building source
    monkeypatch.setenv("IROBOTS_CAMERA_SOURCE", "esp32_cam")
    cfg_esp = CameraSourceConfig.from_configs(p_cfg, scene)
    assert cfg_esp.source is CameraSourceMode.ESP32_CAM
    with pytest.raises(ConfigError, match="BOARD_IP"):
        build_camera_source(cfg_esp, sim=None)

    # Unknown camera source mode raises ConfigError
    monkeypatch.setenv("IROBOTS_CAMERA_SOURCE", "webcam_usb")
    with pytest.raises(ConfigError, match="invalid camera source"):
        CameraSourceConfig.from_configs(p_cfg, scene)


def test_sim_camera_source_captures_paused_sim_frame_with_sim_timestamp(
    robot, scene: dict
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
    assert cap1.source is CameraSourceMode.SIM
    assert cap1.frame_id == 1
    assert cap2.frame_id == 2
    assert cap1.timestamp_s == pytest.approx(steps_before * sim.dt)
    assert cap1.rgb.shape == (480, 640, 3)
    assert cap1.rgb.dtype == np.uint8
    assert np.array_equal(cap1.rgb, cap2.rgb)


def test_esp32_camera_source_fetches_jpeg_retries_and_resizes_qvga_to_vga() -> None:
    """T68: ESP32CameraSource fetches /capture over HTTP, retries transient errors, and upscales 320x240 -> 640x480."""
    # Native 320x240 QVGA frame with a bright red square in the centre
    qvga = np.full((240, 320, 3), (97, 99, 107), dtype=np.uint8)
    qvga[100:140, 140:180] = (225, 55, 55)

    with _LocalESP32CamServer(qvga, fail_first_n=1) as srv:
        src = ESP32CameraSource(
            endpoint=srv.endpoint,
            capture_path="/capture",
            width=640,
            height=480,
            timeout_s=2.0,
            retries=3,
            resize_to_scene=True,
        )
        cap = src.capture(sim_time_s=1.25)
        assert srv.requests_seen == 2, "expected 1 failed attempt + 1 retry"
        assert cap.source is CameraSourceMode.ESP32_CAM
        assert cap.timestamp_s == pytest.approx(1.25)
        assert cap.rgb.shape == (480, 640, 3)
        # Centre pixel must still be red (R > 180, B < 90) confirming BGR->RGB conversion
        centre_rgb = cap.rgb[240, 320]
        assert int(centre_rgb[0]) > 180 and int(centre_rgb[2]) < 90


def test_closed_loop_runs_end_to_end_with_both_sim_and_esp32_camera_sources(
    robot, scene: dict, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """T69: full ClosedLoopRunner completes L1 on both `sim` and `esp32_cam` sources via config/perception.yaml."""
    # 1. Run with `sim` camera source
    monkeypatch.setenv("IROBOTS_CAMERA_SOURCE", "sim")
    runner_sim = ClosedLoopRunner(robot, scene)
    assert runner_sim.camera_source_config.source is CameraSourceMode.SIM
    res_sim = runner_sim.run_episode(
        task_path="config/tasks/l1.yaml",
        seed=3,
        jsonl_path=tmp_path / "ep_sim.jsonl",
    )
    assert res_sim.task_success_privileged is True
    assert res_sim.task_success_perceived is True
    assert res_sim.camera_source == "sim"

    # 2. Run with `esp32_cam` camera source backed by the local HTTP `/capture` server
    task = load_task_spec("config/tasks/l1.yaml")
    seeded = build_seeded_scene(robot, scene, task, seed=3)
    preview_sim = DeterministicKinematicSim(robot, scene, seeded.to_sim_objects())
    preview_sim.reset(dict(robot.home_pose), robot.gripper.max_opening)
    preview_sim.step(50)
    initial_frame = preview_sim.render()
    preview_sim.close()

    with _LocalESP32CamServer(initial_frame) as srv:
        monkeypatch.setenv("IROBOTS_CAMERA_SOURCE", "esp32_cam")
        monkeypatch.setenv("BOARD_IP", srv.endpoint)
        runner_esp = ClosedLoopRunner(robot, scene)
        assert runner_esp.camera_source_config.source is CameraSourceMode.ESP32_CAM
        res_esp = runner_esp.run_episode(
            task_path="config/tasks/l1.yaml",
            seed=3,
            jsonl_path=tmp_path / "ep_esp.jsonl",
        )
        assert srv.requests_seen >= 3
        assert res_esp.task_success_privileged is True
        assert res_esp.camera_source == "esp32_cam"
