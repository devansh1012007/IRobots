"""Stdlib HTTP server and single-source-of-truth backend for the IRobots v2 Web Control Console (PLAN:irobots-v2:C66, C67-F4).

Serves the offline-first, gradient-free deep-black/grey operator control interface
(``web/irobots_control_console.html``) and exposes JSON/JPEG API endpoints backed
exclusively by ``irobots_core``:
- ``GET  /api/state``          full state: pose, tcp, tracks, plan, safety verdict, estop, camera_source, episode summary, action_log_sha256
- ``GET  /api/frame.jpg``      latest camera frame (sim render or esp32 frame) as JPEG
- ``GET  /api/config``         robot limits, scene, camera spec read from ``config/*.yaml``
- ``POST /api/camera_source``  ``{"source": "sim"|"esp32_cam"}``; esp32 rules from F3 apply and errors are returned as JSON
- ``POST /api/task``           ``{"text": "..."}`` -> planner -> plan returned for approval; validated against current scene (T40)
- ``POST /api/approve``        approve the plan (plan-then-autonomous, D11)
- ``POST /api/step``           execute exactly ONE skill (I3) through ``ClosedLoopRunner`` machinery
- ``POST /api/run``            run to completion with per-skill and per-episode deadlines (I4)
- ``POST /api/estop``          immediate halt, works while a run is in progress (thread-safe ``EStopController``)
- ``POST /api/reset``          new seeded episode ``{seed, level}``
"""

from __future__ import annotations

import argparse
import http.server
import ipaddress
import json
import logging
import math
import threading
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from irobots_core.config import REPO_ROOT, ConfigError, env_or, load_yaml
from irobots_core.estop import EStopController
from irobots_core.kinematics import forward_kinematics
from irobots_core.loop import ClosedLoopRunner, EpisodeSession
from irobots_core.perception.camera_source import CameraSourceMode
from irobots_core.robot_model import load_robot
from irobots_core.safety import _chain_keypoints
from irobots_core.skills import SkillValidationError

LOGGER = logging.getLogger(__name__)


def _is_loopback_host(host: str) -> bool:
    clean = host.strip().lower()
    if clean == "localhost":
        return True
    try:
        return ipaddress.ip_address(clean).is_loopback
    except ValueError:
        return False


class WebConsoleBackend:
    """Pure-Python state & API authority backing the Web Control Console (C66, C67-F4)."""

    def __init__(
        self,
        repo_root: Path = REPO_ROOT,
        *,
        step_delay_s: float = 0.0,
    ) -> None:
        self.repo_root = repo_root
        self.html_path = repo_root / "web" / "irobots_control_console.html"
        self.robot = load_robot(repo_root / "config" / "robot.yaml")
        self.scene_cfg = load_yaml(repo_root / "config" / "scene.yaml")["scene"]
        self.perception_cfg = load_yaml(repo_root / "config" / "perception.yaml")["perception"]
        self.estop = EStopController()
        self.step_delay_s = float(step_delay_s)
        self._lock = threading.RLock()
        self.run_in_progress: bool = False
        self.fw_seq: int = 1
        self.chat_log: list[dict[str, Any]] = []
        self.ndjson_log: list[dict[str, Any]] = []

        self.runner = ClosedLoopRunner(
            self.robot,
            self.scene_cfg,
            estop=self.estop,
            camera_source_override="sim",
        )
        self.session: EpisodeSession = self._create_session(seed=0, level="L1")

    def _task_path_for_level(self, level: str) -> Path:
        lvl = level.strip().upper()
        if lvl == "L2":
            return self.repo_root / "config" / "tasks" / "l2.yaml"
        if lvl == "L1":
            return self.repo_root / "config" / "tasks" / "l1.yaml"
        raise ValueError(f"unsupported task level {level!r}; expected 'L1' or 'L2'")

    def _append_chat(self, role: str, text: str, tone: str = "normal") -> None:
        sim_t = round(self.session.sim.total_steps * self.session.sim.dt, 3) if hasattr(self, "session") else 0.1
        self.chat_log.append(
            {
                "role": role,
                "text": text,
                "tone": tone,
                "sim_time_s": sim_t,
            }
        )

    def _append_ndjson(self, cmd: dict[str, Any], ack: dict[str, Any]) -> None:
        self.ndjson_log.append({"cmd": cmd, "ack": ack})

    def _create_session(self, *, seed: int, level: str) -> EpisodeSession:
        self.estop.reset()
        t_path = self._task_path_for_level(level)
        sess = self.runner.start_episode(
            task_path=t_path,
            seed=seed,
            auto_approve_plan=False,
        )
        sess.observe_scene()
        try:
            sess.propose_plan_from_text(sess.task.instruction)
        except ValueError:
            pass
        self.chat_log.clear()
        self.ndjson_log.clear()
        self._append_chat(
            "SYSTEM",
            f"Initialized {level.upper()} (seed #{seed}) on camera_source={self.runner.camera_source_config.source.value}. "
            f"Tracked {len(sess.latest_tracks)} object(s).",
            "normal",
        )
        self._append_chat(
            "PLANNER",
            f"Proposed {len(sess.proposed_plan)}-step coordinate-free skill plan. Approval required before motion (D11).",
            "warn",
        )
        return sess

    def load_html(self) -> str:
        return self.html_path.read_text(encoding="utf-8")

    def get_config_payload(self) -> dict[str, Any]:
        """Return robot limits, links, scene, and camera config from YAML (single source of truth)."""
        return {
            "robot": {
                "name": self.robot.name,
                "links": {
                    "base_height": {"value": float(self.robot.base_height), "provenance": self.robot.base_height.provenance},
                    "upper_arm": {"value": float(self.robot.upper_arm), "provenance": self.robot.upper_arm.provenance},
                    "fore_arm": {"value": float(self.robot.fore_arm), "provenance": self.robot.fore_arm.provenance},
                    "wrist_post": {"value": float(self.robot.wrist_post), "provenance": self.robot.wrist_post.provenance},
                    "gripper_link": {"value": float(self.robot.gripper_link), "provenance": self.robot.gripper_link.provenance},
                    "grip_thickness": {"value": float(self.robot.grip_thickness), "provenance": self.robot.grip_thickness.provenance},
                },
                "joints": {
                    name: {
                        "min_deg": spec.min_deg,
                        "max_deg": spec.max_deg,
                        "max_velocity": spec.max_velocity,
                        "max_acceleration": spec.max_acceleration,
                        "channel": self.robot.servo_channels.get(name, 0),
                        "provenance": spec.provenance,
                    }
                    for name, spec in self.robot.joints.items()
                },
                "gripper": {
                    "min_opening": self.robot.gripper.min_opening,
                    "max_opening": self.robot.gripper.max_opening,
                    "channel": self.robot.servo_channels.get("gripper", 5),
                    "provenance": self.robot.gripper.provenance,
                },
                "safety": {
                    "workspace_centre": list(self.robot.safety.workspace_centre),
                    "workspace_radius": self.robot.safety.workspace_radius,
                    "z_min": self.robot.safety.z_min,
                    "z_max": self.robot.safety.z_max,
                    "table_plane_z": self.robot.safety.table_plane_z,
                    "table_clearance": self.robot.safety.table_clearance,
                    "self_collision_radius": self.robot.safety.self_collision_radius,
                },
                "home_pose": dict(self.robot.home_pose),
            },
            "scene": {
                "table": self.scene_cfg["table"],
                "workspace": self.scene_cfg["workspace"],
                "base_position": list(self.scene_cfg.get("base_position", [0.0, 0.0, 0.0])),
            },
            "camera": self.scene_cfg["camera"],
        }

    def set_camera_source(
        self,
        source: str,
        *,
        calibration_file: str | Path | None = None,
        allow_real_camera_with_sim_arm: bool | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            current_src = self.runner.camera_source_config.source.value
            try:
                new_runner = ClosedLoopRunner(
                    self.robot,
                    self.scene_cfg,
                    estop=self.estop,
                    camera_source_override=source.strip().lower(),
                    calibration_file=calibration_file,
                    allow_real_camera_with_sim_arm=allow_real_camera_with_sim_arm,
                )
                self.runner = new_runner
                self.session = self._create_session(
                    seed=self.session.seed,
                    level=self.session.task.level,
                )
                return {
                    "ok": True,
                    "camera_source": self.runner.camera_source_config.source.value,
                    "execution_mode": self.session.execution_mode,
                }
            except (ConfigError, ValueError) as exc:
                self._append_chat("CAMERA [F3]", f"Refused camera_source switch: {exc}", "error")
                return {
                    "ok": False,
                    "error": str(exc),
                    "camera_source": current_src,
                }

    def propose_task(self, text: str) -> dict[str, Any]:
        with self._lock:
            clean = text.strip()
            if not clean:
                return {"ok": False, "error": "Task text must not be empty"}
            self._append_chat("OPERATOR", clean, "normal")
            try:
                plan = self.session.propose_plan_from_text(clean)
            except (ValueError, SkillValidationError) as exc:
                self._append_chat("PLANNER [T40]", str(exc), "error")
                return {"ok": False, "error": str(exc)}

            plan_dicts = [{"skill": c.skill, "target_id": c.target_id or ""} for c in plan]
            self._append_chat(
                "PLANNER [C31]",
                f"Generated {len(plan_dicts)}-step plan ({', '.join(f'{c.skill}({c.target_id or str()})' for c in plan)}). Awaiting operator approval.",
                "normal",
            )
            return {"ok": True, "plan": plan_dicts, "plan_approved": self.session.plan_approved}

    def approve_plan(self) -> dict[str, Any]:
        with self._lock:
            if not self.session.proposed_plan and not self.session.remaining_plan:
                return {"ok": False, "error": "No plan to approve"}
            self.session.approve_plan()
            self._append_chat("OPERATOR [D11]", "Approved skill plan for execution.", "success")
            return {"ok": True, "plan_approved": True}

    def step_once(self) -> dict[str, Any]:
        with self._lock:
            if self.estop.triggered:
                return {"ok": False, "error": "E-Stop is latched; clear E-Stop before stepping"}
            if not self.session.plan_approved:
                return {"ok": False, "error": "Plan must be approved before stepping (D11)"}
            if self.session.done:
                return {
                    "ok": True,
                    "done": True,
                    "action_log_sha256": self.session.current_summary().action_log_sha256,
                }

            rec = self.session.step_turn()
            pose_now = self.session.sim.joint_positions()
            seq = self.fw_seq
            self.fw_seq += 1
            self._append_ndjson(
                {
                    "cmd": "move",
                    "seq": seq,
                    "joints_deg": {k: round(math.degrees(v), 1) for k, v in pose_now.items()},
                    "gripper_m": round(self.session.sim.opening(), 3),
                },
                {"ok": rec.get("safety_approved", True), "seq": seq, "state": "ACTIVE"},
            )
            outcome = str(rec.get("outcome", "done"))
            skill_name = str(rec.get("skill", "step"))
            target_id = str(rec.get("target_id", ""))
            tone = (
                "error"
                if outcome.startswith("safety_rejected") or outcome in ("camera_unavailable", "estop_aborted")
                else ("success" if outcome == "object_lifted" else "normal")
            )
            self._append_chat(
                "WORKER [I3]",
                f"Executed 1 skill {skill_name}({target_id}) -> outcome={outcome}",
                tone,
            )
            summary = self.session.current_summary()
            return {
                "ok": True,
                "turn": rec,
                "done": self.session.done,
                "action_log_sha256": summary.action_log_sha256,
            }

    def run_to_completion(self) -> dict[str, Any]:
        with self._lock:
            if self.estop.triggered:
                return {"ok": False, "error": "E-Stop is latched; clear E-Stop before running"}
            if not self.session.plan_approved:
                return {"ok": False, "error": "Plan must be approved before running (D11)"}
            self.run_in_progress = True

        try:
            while True:
                if self.estop.triggered:
                    with self._lock:
                        self.session.failure_mode = "estop_aborted"
                        self.session.done = True
                    break
                with self._lock:
                    if self.session.done:
                        break
                    self.step_once()
                    done_now = self.session.done
                if done_now:
                    break
                if self.step_delay_s > 0.0:
                    t_end = time.monotonic() + self.step_delay_s
                    while time.monotonic() < t_end:
                        if self.estop.triggered:
                            with self._lock:
                                self.session.failure_mode = "estop_aborted"
                                self.session.done = True
                            break
                        time.sleep(min(0.01, max(0.0, t_end - time.monotonic())))
        finally:
            with self._lock:
                self.run_in_progress = False
                summary = self.session.current_summary()

        return {
            "ok": True,
            "done": self.session.done,
            "episode_summary": self._serialize_summary(summary),
            "action_log_sha256": summary.action_log_sha256,
        }

    def trigger_estop(self, estop: bool = True) -> dict[str, Any]:
        # Intentionally does NOT wait on self._lock before latching self.estop so an
        # in-progress step_guarded() halts within one 2 ms physics cycle (T52).
        if estop:
            self.estop.trigger()
            self.session.failure_mode = "estop_aborted"
            self.session.done = True
            seq = self.fw_seq
            self.fw_seq += 1
            self._append_ndjson({"cmd": "estop", "seq": seq}, {"ok": True, "seq": seq, "state": "ESTOP_HOLD"})
            self._append_chat("E-STOP [C39]", "Immediate E-Stop latched; holding pose and aborting episode.", "error")
        else:
            self.estop.reset()
            seq = self.fw_seq
            self.fw_seq += 1
            self._append_ndjson({"cmd": "resume", "seq": seq}, {"ok": True, "seq": seq, "state": "IDLE"})
            self._append_chat("E-STOP [C39]", "E-Stop cleared; arm returned to IDLE.", "success")
        return {"ok": True, "estop": self.estop.triggered}

    def reset_episode(self, *, seed: int = 0, level: str = "L1") -> dict[str, Any]:
        with self._lock:
            self.session = self._create_session(seed=seed, level=level)
            return {
                "ok": True,
                "seed": self.session.seed,
                "level": self.session.task.level,
                "episode_id": self.session.episode_id,
            }

    def save_current_episode_jsonl(self, path: str | Path) -> Path:
        with self._lock:
            out = Path(path)
            summary = self.session.current_summary()
            out.parent.mkdir(parents=True, exist_ok=True)
            lines = [json.dumps(r, sort_keys=True) for r in self.session.turn_records]
            lines.append(
                json.dumps(
                    {
                        "type": "episode_summary",
                        "episode_id": summary.episode_id,
                        "action_log_sha256": summary.action_log_sha256,
                        "task_success_privileged": summary.task_success_privileged,
                        "task_success_perceived": summary.task_success_perceived,
                        "camera_source": summary.camera_source,
                        "execution_mode": summary.execution_mode,
                        "failure_mode": summary.failure_mode,
                    },
                    sort_keys=True,
                )
            )
            out.write_text("\n".join(lines) + "\n", encoding="utf-8")
            return out

    @staticmethod
    def _serialize_summary(summary: Any) -> dict[str, Any]:
        return {
            "episode_id": summary.episode_id,
            "seed": summary.seed,
            "task_name": summary.task_name,
            "task_success_privileged": summary.task_success_privileged,
            "task_success_perceived": summary.task_success_perceived,
            "skills_executed": summary.skills_executed,
            "observations_taken": summary.observations_taken,
            "max_skills_per_observation": summary.max_skills_per_observation,
            "safety_rejections": summary.safety_rejections,
            "sim_seconds": round(float(summary.sim_seconds), 4),
            "action_log_sha256": summary.action_log_sha256,
            "failure_mode": summary.failure_mode,
            "camera_source": summary.camera_source,
            "execution_mode": summary.execution_mode,
        }

    def get_latest_jpeg(self) -> bytes:
        with self._lock:
            if self.session.latest_frame_rgb is None:
                self.session.observe_scene()
            rgb = (
                self.session.latest_frame_rgb.copy()
                if self.session.latest_frame_rgb is not None
                else np.zeros((480, 640, 3), dtype=np.uint8)
            )
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
            if not ok:
                raise RuntimeError("failed to encode camera frame as JPEG")
            return bytes(buf)

    def get_state_snapshot(self) -> dict[str, Any]:
        with self._lock:
            pose = self.session.sim.joint_positions()
            opening = self.session.sim.opening()
            fk = forward_kinematics(self.robot, pose)
            elbow, wrist, tip = _chain_keypoints(self.robot, pose)
            base_h = float(self.robot.base_height)
            shoulder = [0.0, 0.0, base_h]
            half_open = opening / 2.0
            c_az = math.cos(fk.azimuth)
            s_az = math.sin(fk.azimuth)
            f_pos = [float(tip[0] + half_open * c_az), float(tip[1] + half_open * s_az), float(tip[2])]
            f_neg = [float(tip[0] - half_open * c_az), float(tip[1] - half_open * s_az), float(tip[2])]

            summary = self.session.current_summary()
            src_mode = self.runner.camera_source_config.source
            if src_mode is CameraSourceMode.ESP32_CAM:
                badge = "HYBRID" if self.runner.arm_backend == "mujoco" else "REAL CAMERA"
            else:
                badge = "SIM"

            done_count = len(self.session.proposed_plan) - len(self.session.remaining_plan)
            plan_items: list[dict[str, Any]] = []
            for idx, call in enumerate(self.session.proposed_plan):
                if idx < done_count:
                    st_str = "done"
                elif idx == done_count and not self.session.done:
                    st_str = "next"
                else:
                    st_str = "wait"
                plan_items.append(
                    {
                        "index": idx,
                        "skill": call.skill,
                        "target_id": call.target_id or "",
                        "status": st_str,
                    }
                )

            sim_objs: list[dict[str, Any]] = []
            for o in self.session.seeded.to_sim_objects():
                oname = str(o["name"])
                opos = self.session.sim.object_position(oname)
                sim_objs.append(
                    {
                        "name": oname,
                        "size": [float(c) for c in o.get("size", [0.04, 0.04, 0.04])],
                        "pos": [round(float(opos[0]), 4), round(float(opos[1]), 4), round(float(opos[2]), 4)],
                        "is_bin": bool(o.get("is_bin", False)),
                    }
                )

            verdict = self.session.last_verdict
            return {
                "robot_name": self.robot.name,
                "camera_source": src_mode.value,
                "execution_mode": self.session.execution_mode,
                "badge": badge,
                "estop": self.estop.triggered,
                "estopped": self.estop.triggered,
                "run_in_progress": self.run_in_progress,
                "plan_approved": self.session.plan_approved,
                "sim_time_s": round(self.session.sim.total_steps * self.session.sim.dt, 3),
                "sim_steps": self.session.sim.total_steps,
                "pose": {name: round(float(pose[name]), 5) for name in self.robot.joints},
                "joints_deg": {
                    name: round(math.degrees(pose[name]), 2) for name in self.robot.joints
                },
                "gripper_opening_m": round(float(opening), 4),
                "tcp_xyz_m": [round(float(c), 4) for c in fk.position],
                "tcp": {
                    "position": [round(float(c), 4) for c in fk.position],
                    "azimuth_deg": round(math.degrees(fk.azimuth), 2),
                    "opening_m": round(float(opening), 4),
                    "keypoints_xyz": {
                        "base": [0.0, 0.0, 0.0],
                        "shoulder": [round(c, 4) for c in shoulder],
                        "elbow": [round(float(c), 4) for c in elbow],
                        "wrist": [round(float(c), 4) for c in wrist],
                        "tip": [round(float(c), 4) for c in tip],
                        "finger_pos": [round(c, 4) for c in f_pos],
                        "finger_neg": [round(c, 4) for c in f_neg],
                    },
                },
                "tracks": list(self.session.latest_track_details),
                "objects": sim_objs,
                "plan": plan_items,
                "safety_verdict": {
                    "approved": verdict.approved,
                    "reason": verdict.reason.value,
                    "detail": verdict.detail,
                    "rejections": self.session.supervisor.rejected_infeasible,
                },
                "safety_rejections": self.session.supervisor.rejected_infeasible,
                "predicates": dict(self.session.last_predicates),
                "episode_summary": self._serialize_summary(summary),
                "turn_records": list(self.session.turn_records),
                "action_log_sha256": summary.action_log_sha256,
                "chat_log": list(self.chat_log),
                "ndjson_log": list(self.ndjson_log),
            }


class WebConsoleServer:
    """Stdlib ThreadingHTTPServer exposing the WebConsoleBackend JSON & JPEG API (C67-F4)."""

    def __init__(
        self,
        backend: WebConsoleBackend,
        *,
        host: str | None = None,
        port: int | None = None,
        allow_remote_bind: bool | None = None,
    ) -> None:
        self.backend = backend
        web_cfg = backend.perception_cfg.get("web_console") or {}
        resolved_host = (
            host
            if host is not None
            else (env_or("IROBOTS_WEB_HOST") or str(web_cfg.get("host", "localhost")))
        ).strip()
        resolved_port = (
            int(port)
            if port is not None
            else int(env_or("IROBOTS_WEB_PORT") or web_cfg.get("port", 0))
        )
        remote_ok = (
            bool(allow_remote_bind)
            if allow_remote_bind is not None
            else bool(web_cfg.get("allow_remote_bind", False))
        )
        self.max_body_bytes = int(web_cfg.get("max_body_bytes", 65536))

        if not _is_loopback_host(resolved_host):
            if not remote_ok:
                raise ConfigError(
                    f"Refusing to bind WebConsoleServer to non-loopback host {resolved_host!r} "
                    "without perception.web_console.allow_remote_bind=True"
                )
            LOGGER.warning(
                "Binding WebConsoleServer to non-loopback host %s with allow_remote_bind=True",
                resolved_host,
            )

        parent = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def _send_json(self, status: int, payload: dict[str, Any]) -> None:
                body = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _read_json_body(self) -> tuple[int, dict[str, Any] | None]:
                raw_len = self.headers.get("Content-Length", "0")
                try:
                    length = int(raw_len)
                except ValueError:
                    return 400, {"ok": False, "error": "Invalid Content-Length header"}
                if length > parent.max_body_bytes:
                    return 413, {
                        "ok": False,
                        "error": f"Request body ({length} bytes) exceeds {parent.max_body_bytes} byte limit",
                    }
                raw = self.rfile.read(length) if length > 0 else b"{}"
                try:
                    parsed = json.loads(raw.decode("utf-8"))
                except Exception:  # noqa: BLE001
                    return 400, {"ok": False, "error": "Malformed JSON request body"}
                if not isinstance(parsed, dict):
                    return 400, {"ok": False, "error": "JSON body must be an object"}
                return 200, parsed

            def do_GET(self) -> None:  # noqa: N802
                path = self.path.split("?", 1)[0]
                try:
                    if path in ("/", "/index.html"):
                        html_bytes = parent.backend.load_html().encode("utf-8")
                        self.send_response(200)
                        self.send_header("Content-Type", "text/html; charset=utf-8")
                        self.send_header("Content-Length", str(len(html_bytes)))
                        self.end_headers()
                        self.wfile.write(html_bytes)
                        return
                    if path == "/api/state":
                        self._send_json(200, parent.backend.get_state_snapshot())
                        return
                    if path == "/api/config":
                        self._send_json(200, parent.backend.get_config_payload())
                        return
                    if path == "/api/frame.jpg":
                        jpg = parent.backend.get_latest_jpeg()
                        self.send_response(200)
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(jpg)))
                        self.send_header("Cache-Control", "no-store")
                        self.end_headers()
                        self.wfile.write(jpg)
                        return
                    self._send_json(404, {"ok": False, "error": f"Unknown endpoint {path!r}"})
                except Exception as exc:  # noqa: BLE001
                    self._send_json(500, {"ok": False, "error": str(exc)})

            def do_POST(self) -> None:  # noqa: N802
                path = self.path.split("?", 1)[0]
                status, body = self._read_json_body()
                if status != 200 or body is None:
                    self._send_json(status, body or {"ok": False, "error": "Invalid request"})
                    return

                try:
                    if path == "/api/camera_source":
                        src = body.get("source")
                        if not isinstance(src, str) or not src.strip():
                            self._send_json(400, {"ok": False, "error": "Field 'source' must be a non-empty string"})
                            return
                        cal_file = body.get("calibration_file")
                        if cal_file is not None and not isinstance(cal_file, str):
                            self._send_json(400, {"ok": False, "error": "Field 'calibration_file' must be a string"})
                            return
                        allow_hyb = body.get("allow_real_camera_with_sim_arm")
                        if allow_hyb is not None and not isinstance(allow_hyb, bool):
                            self._send_json(
                                400, {"ok": False, "error": "Field 'allow_real_camera_with_sim_arm' must be a boolean"}
                            )
                            return
                        res = parent.backend.set_camera_source(
                            src,
                            calibration_file=cal_file,
                            allow_real_camera_with_sim_arm=allow_hyb,
                        )
                        self._send_json(200 if res["ok"] else 400, res)
                        return

                    if path == "/api/task":
                        text = body.get("text")
                        if not isinstance(text, str) or not text.strip():
                            self._send_json(400, {"ok": False, "error": "Field 'text' must be a non-empty string"})
                            return
                        res = parent.backend.propose_task(text)
                        self._send_json(200 if res["ok"] else 400, res)
                        return

                    if path == "/api/approve":
                        res = parent.backend.approve_plan()
                        self._send_json(200 if res["ok"] else 400, res)
                        return

                    if path == "/api/step":
                        res = parent.backend.step_once()
                        self._send_json(200 if res["ok"] else 400, res)
                        return

                    if path == "/api/run":
                        res = parent.backend.run_to_completion()
                        self._send_json(200 if res["ok"] else 400, res)
                        return

                    if path == "/api/estop":
                        estop_flag = body.get("estop", True)
                        if not isinstance(estop_flag, bool):
                            self._send_json(400, {"ok": False, "error": "Field 'estop' must be a boolean"})
                            return
                        res = parent.backend.trigger_estop(estop=estop_flag)
                        self._send_json(200, res)
                        return

                    if path == "/api/reset":
                        seed = body.get("seed", 0)
                        level = body.get("level", "L1")
                        if isinstance(seed, bool) or not isinstance(seed, int):
                            self._send_json(400, {"ok": False, "error": "Field 'seed' must be an integer"})
                            return
                        if not isinstance(level, str) or level.strip().upper() not in ("L1", "L2"):
                            self._send_json(400, {"ok": False, "error": "Field 'level' must be 'L1' or 'L2'"})
                            return
                        res = parent.backend.reset_episode(seed=seed, level=level)
                        self._send_json(200, res)
                        return

                    self._send_json(404, {"ok": False, "error": f"Unknown endpoint {path!r}"})
                except Exception as exc:  # noqa: BLE001
                    self._send_json(400, {"ok": False, "error": str(exc)})

            def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                return

        self._httpd = http.server.ThreadingHTTPServer((resolved_host, resolved_port), Handler)
        addr = self._httpd.server_address
        self.host = str(addr[0])
        self.port = int(addr[1])
        self._thread: threading.Thread | None = None

    @property
    def base_url(self) -> str:
        return f"http://{self.host}:{self.port}"

    def start(self) -> None:
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def serve_forever(self) -> None:
        self._httpd.serve_forever()

    def __enter__(self) -> WebConsoleServer:
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.stop()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="IRobots v2 Web Control Console Server")
    parser.add_argument("--host", type=str, default=None, help="Bind host (default from config/perception.yaml)")
    parser.add_argument("--port", type=int, default=None, help="Bind port (default from config/perception.yaml)")
    args = parser.parse_args(argv)

    backend = WebConsoleBackend()
    server = WebConsoleServer(backend, host=args.host, port=args.port)
    print(f"IRobots v2 Web Console listening on {server.base_url}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
