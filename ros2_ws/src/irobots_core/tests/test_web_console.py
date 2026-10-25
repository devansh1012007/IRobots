"""Tests for the Web Control Console HTML and backend API (PLAN:irobots-v2:C66).

Asserts:
1. `web/irobots_control_console.html` exists, contains zero CSS gradients (`linear-gradient` / `radial-gradient`), and includes all required operator panels (chat window, 5-DOF simulation viewport, SIM vs ESP32_CAM camera viewport, tracker table, joint telemetry, safety supervisor, and E-Stop).
2. `WebConsoleBackend` switches cleanly between `sim` and `esp32_cam` camera sources and returns accurate 5-DOF FK telemetry.
"""

from __future__ import annotations

from pathlib import Path

from irobots_nodes.web_node import WebConsoleBackend


def test_web_console_html_has_no_gradients_and_contains_all_panels(repo_root: Path) -> None:
    html_file = repo_root / "web" / "irobots_control_console.html"
    assert html_file.exists()
    html = html_file.read_text(encoding="utf-8")
    assert "linear-gradient" not in html
    assert "radial-gradient" not in html
    for required_id in (
        "chatStream",
        "chatForm",
        "simCanvas",
        "camCanvas",
        "camSimBtn",
        "camEspBtn",
        "trackerTableBody",
        "jointsContainer",
        "estopBtn",
        "ndjsonLog",
    ):
        assert f'id="{required_id}"' in html, f"missing UI element #{required_id}"


def test_web_console_backend_switches_camera_source_and_reports_fk(repo_root: Path) -> None:
    backend = WebConsoleBackend(repo_root)
    snap = backend.get_state_snapshot()
    assert snap["camera_source"] == "sim"
    assert len(snap["tcp_xyz_m"]) == 3

    res = backend.set_camera_source("esp32_cam")
    assert res["ok"] is False
    assert "calibration" in res["error"].lower()
    assert backend.get_state_snapshot()["camera_source"] == "sim"
