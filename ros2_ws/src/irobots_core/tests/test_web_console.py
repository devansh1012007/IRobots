"""Acceptance tests for the thin-client Web Control Console and stdlib HTTP server (PLAN:irobots-v2:C66, C67-F4).

Asserts:
1. `WebConsoleServer` starts on an ephemeral port in a thread and serves `/`, `/api/config`, `/api/state`, `/api/frame.jpg`, `/api/camera_source`, `/api/task`, `/api/approve`, `/api/step`, `/api/run`, `/api/estop`, and `/api/reset`; invalid JSON -> 400; unknown path -> 404; oversize body (>64 KB) -> 413.
2. `POST /api/task` referencing an absent object returns a 400 rejection (T40).
3. `POST /api/step` twice never executes two skills for one observation (Invariant I3).
4. `POST /api/estop` during `POST /api/run` halts within one control cycle and marks the episode aborted (`estop_aborted`).
5. `POST /api/camera_source` with `esp32_cam` without calibration returns a 400 error JSON and leaves state unchanged.
6. `web/irobots_control_console.html` contains no `"http://"` or `"https://"` external URLs, no `"linear-gradient"` or `"radial-gradient"`, and no copy of `forwardKinematics`, `inverseKinematics`, or `checkSafety`.
7. `action_log_sha256` returned by `/api/state` equals the hash `replay.py` computes from the recorded `episode.jsonl`.
"""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

from irobots_core.replay import compute_action_log_sha256, replay_episode
from irobots_nodes.web_node import WebConsoleBackend, WebConsoleServer


def _request_json(
    base_url: str,
    path: str,
    *,
    method: str = "GET",
    body: dict[str, Any] | bytes | None = None,
) -> tuple[int, dict[str, Any]]:
    data: bytes | None = None
    headers: dict[str, str] = {}
    if isinstance(body, dict):
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    elif isinstance(body, bytes):
        data = body
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(f"{base_url}{path}", data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=10.0) as resp:  # noqa: S310
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        payload = json.loads(exc.read().decode("utf-8"))
        return exc.code, payload


def test_web_console_html_is_offline_first_gradient_free_and_has_no_js_kinematics_or_safety(
    repo_root: Path,
) -> None:
    """C67-F4: HTML contains no external http(s) URLs, no gradients, and no duplicate FK/IK/safety functions."""
    html_file = repo_root / "web" / "irobots_control_console.html"
    assert html_file.exists()
    html = html_file.read_text(encoding="utf-8")

    assert "http://" not in html
    assert "https://" not in html
    assert "linear-gradient" not in html
    assert "radial-gradient" not in html
    for forbidden_symbol in ("forwardKinematics", "inverseKinematics", "checkSafety"):
        assert forbidden_symbol not in html, f"JS must not duplicate {forbidden_symbol}"

    for required_id in (
        "connectionBanner",
        "modeBadge",
        "chatStream",
        "chatForm",
        "simCanvas",
        "camCanvas",
        "camSimBtn",
        "camEspBtn",
        "trackerTableBody",
        "jointsContainer",
        "estopBtn",
        "approvePlanBtn",
        "stepSkillBtn",
        "runLoopBtn",
        "ndjsonLog",
    ):
        assert f'id="{required_id}"' in html, f"missing UI element #{required_id}"


def test_web_console_server_endpoints_error_codes_and_absent_object_rejection(
    repo_root: Path,
) -> None:
    """C67-F4: test every HTTP endpoint, 400 invalid JSON, 404 unknown path, 413 oversize body, T40 absent object, and F3 esp32_cam refusal."""
    backend = WebConsoleBackend(repo_root)
    with WebConsoleServer(backend, port=0) as srv:
        base = srv.base_url

        # 1. GET / serves HTML
        with urllib.request.urlopen(f"{base}/", timeout=5.0) as resp:  # noqa: S310
            assert resp.status == 200
            html_bytes = resp.read()
            assert b"IROBOTS" in html_bytes

        # 2. GET /api/config returns robot, scene, and camera specs from YAML
        code, cfg = _request_json(base, "/api/config")
        assert code == 200
        assert cfg["robot"]["name"] == "irobots_arm_5dof"
        assert "joints" in cfg["robot"]
        assert cfg["camera"]["width"] == 640

        # 3. GET /api/state returns full state snapshot
        code, st = _request_json(base, "/api/state")
        assert code == 200
        assert st["camera_source"] == "sim"
        assert st["badge"] == "SIM"
        assert len(st["tcp"]["position"]) == 3
        assert len(st["tracks"]) >= 1

        # 4. GET /api/frame.jpg returns valid JPEG bytes (SOI 0xFFD8 .. EOI 0xFFD9)
        with urllib.request.urlopen(f"{base}/api/frame.jpg", timeout=5.0) as f_resp:  # noqa: S310
            assert f_resp.status == 200
            assert f_resp.headers.get("Content-Type") == "image/jpeg"
            jpg = f_resp.read()
            assert jpg[:2] == b"\xff\xd8" and jpg[-2:] == b"\xff\xd9"

        # 5. Unknown path -> 404 JSON error
        code, err_404 = _request_json(base, "/api/does_not_exist")
        assert code == 404
        assert err_404["ok"] is False

        # 6. Invalid JSON -> 400 JSON error (never a stack trace)
        code, err_400 = _request_json(base, "/api/task", method="POST", body=b"{not-valid-json")
        assert code == 400
        assert err_400["ok"] is False
        assert "Traceback" not in json.dumps(err_400)

        # 7. Oversize body (>64 KB) -> 413 JSON error
        huge_body = b'{"text":"' + (b"x" * 70000) + b'"}'
        code, err_413 = _request_json(base, "/api/task", method="POST", body=huge_body)
        assert code == 413
        assert err_413["ok"] is False

        # 8. POST /api/camera_source esp32_cam without calibration -> 400 error & state unchanged
        code, cam_err = _request_json(
            base, "/api/camera_source", method="POST", body={"source": "esp32_cam"}
        )
        assert code == 400
        assert cam_err["ok"] is False
        _, st_after = _request_json(base, "/api/state")
        assert st_after["camera_source"] == "sim"

        # 9. POST /api/task referencing an absent object -> 400 rejection (T40)
        code, task_err = _request_json(
            base, "/api/task", method="POST", body={"text": "Pick up the purple block"}
        )
        assert code == 400
        assert task_err["ok"] is False
        assert "PlanApprovalError" in task_err["error"]


def test_web_console_step_enforces_i3_and_matches_replay_sha256(
    repo_root: Path, tmp_path: Path
) -> None:
    """C67-F4 (I3, C38): /api/step requires approval, executes 1 skill per observation, and matches replay.py SHA-256."""
    backend = WebConsoleBackend(repo_root)
    with WebConsoleServer(backend, port=0) as srv:
        base = srv.base_url

        # Reset to seed 0, L1
        code, reset_res = _request_json(
            base, "/api/reset", method="POST", body={"seed": 0, "level": "L1"}
        )
        assert code == 200
        assert reset_res["ok"] is True

        # Propose valid task plan
        code, task_res = _request_json(
            base,
            "/api/task",
            method="POST",
            body={"text": "Pick up the red block and lift it"},
        )
        assert code == 200
        assert task_res["ok"] is True
        assert len(task_res["plan"]) == 3

        # Stepping before approval is rejected (D11)
        code, unapproved = _request_json(base, "/api/step", method="POST", body={})
        assert code == 400
        assert unapproved["ok"] is False

        # Approve plan
        code, app_res = _request_json(base, "/api/approve", method="POST", body={})
        assert code == 200
        assert app_res["ok"] is True

        # Step twice: must take 1 observation per skill (I3)
        code, s1 = _request_json(base, "/api/step", method="POST", body={})
        assert code == 200 and s1["ok"] is True
        code, s2 = _request_json(base, "/api/step", method="POST", body={})
        assert code == 200 and s2["ok"] is True

        _, st_mid = _request_json(base, "/api/state")
        ep_mid = st_mid["episode_summary"]
        assert ep_mid["skills_executed"] == 2
        assert ep_mid["max_skills_per_observation"] == 1
        assert ep_mid["observations_taken"] >= ep_mid["skills_executed"]

        # Step final skill to finish episode and compare action_log_sha256 with replay.py
        code, s3 = _request_json(base, "/api/step", method="POST", body={})
        assert code == 200 and s3["ok"] is True

        _, st_final = _request_json(base, "/api/state")
        api_hash = st_final["action_log_sha256"]
        ep_jsonl = tmp_path / "web_ep.jsonl"
        backend.save_current_episode_jsonl(ep_jsonl)

        rep = replay_episode(ep_jsonl)
        assert rep.reproduced is True
        assert rep.computed_sha256 == api_hash
        assert compute_action_log_sha256(st_final["turn_records"]) == api_hash


def test_web_console_estop_during_run_halts_within_one_cycle_and_marks_aborted(
    repo_root: Path,
) -> None:
    """C67-F4 (T52): POST /api/estop during POST /api/run halts immediately and marks episode estop_aborted."""
    backend = WebConsoleBackend(repo_root, step_delay_s=0.08)
    with WebConsoleServer(backend, port=0) as srv:
        base = srv.base_url
        _request_json(base, "/api/reset", method="POST", body={"seed": 2, "level": "L1"})
        _request_json(
            base, "/api/task", method="POST", body={"text": "Pick up the red block"}
        )
        _request_json(base, "/api/approve", method="POST", body={})

        run_result: dict[str, Any] = {}

        def _call_run() -> None:
            code, payload = _request_json(base, "/api/run", method="POST", body={})
            run_result["code"] = code
            run_result["payload"] = payload

        run_thread = threading.Thread(target=_call_run, daemon=True)
        run_thread.start()
        time.sleep(0.03)

        code, estop_res = _request_json(base, "/api/estop", method="POST", body={"estop": True})
        assert code == 200
        assert estop_res["ok"] is True
        assert estop_res["estop"] is True

        run_thread.join(timeout=5.0)
        assert not run_thread.is_alive()

        _, st = _request_json(base, "/api/state")
        assert st["estop"] is True
        assert st["episode_summary"]["failure_mode"] == "estop_aborted"
