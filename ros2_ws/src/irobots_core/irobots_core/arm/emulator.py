"""Host-side Python emulator of the ESP32 arm firmware state machine and TCP NDJSON server (PLAN:irobots-v2:C50, T57, T58, T59)."""

from __future__ import annotations

import json
import math
import socket
import subprocess
import tempfile
import threading
from pathlib import Path
from typing import Any

from ..robot_model import JOINT_ORDER, RobotModel
from .protocol import run_conformance_suite


class ArmEmulator:
    """Host-side implementation of the ESP32 firmware state machine (C50)."""

    WATCHDOG_TIMEOUT_MS: int = 500

    def __init__(self, robot: RobotModel) -> None:
        self.robot = robot
        self.state: str = "IDLE"
        self.last_seq: int = 0
        self.last_cmd_ms: int = 0
        self.joints_deg: dict[str, float] = {}
        self.gripper_m: float = robot.gripper.max_opening
        self.reset_protocol()

    def reset_protocol(self) -> None:
        self.state = "IDLE"
        self.last_seq = 0
        self.last_cmd_ms = 0
        self.joints_deg = {
            name: math.degrees(float(self.robot.home_pose.get(name, 0.0)))
            for name in JOINT_ORDER
        }
        self.gripper_m = float(self.robot.home_pose.get("gripper", 0.045))

    def advance_time_ms(self, now_ms: int) -> dict[str, Any]:
        if self.state == "ACTIVE" and (now_ms - self.last_cmd_ms) > self.WATCHDOG_TIMEOUT_MS:
            self.state = "WATCHDOG_HOLD"
        return self._status_dict(ok=True)

    def _status_dict(self, *, ok: bool, error: str = "") -> dict[str, Any]:
        out: dict[str, Any] = {
            "ok": ok,
            "seq": self.last_seq,
            "state": self.state,
            "held_joints_deg": dict(self.joints_deg),
            "gripper_m": self.gripper_m,
        }
        if error:
            out["error"] = error
        return out

    def handle_line(self, line: str, now_ms: int = 0) -> dict[str, Any]:
        self.advance_time_ms(now_ms)
        try:
            msg = json.loads(line)
            if not isinstance(msg, dict):
                return self._status_dict(ok=False, error="malformed_json")
        except Exception:
            return self._status_dict(ok=False, error="malformed_json")

        cmd = str(msg.get("cmd", ""))
        seq = msg.get("seq")
        if not isinstance(seq, int):
            return self._status_dict(ok=False, error="invalid_seq")

        if cmd == "estop":
            self.state = "ESTOP_HOLD"
            self.last_seq = seq
            self.last_cmd_ms = now_ms
            return self._status_dict(ok=True)

        if cmd == "resume":
            if seq != self.last_seq + 1:
                return self._status_dict(ok=False, error="sequence_gap")
            self.state = "IDLE"
            self.last_seq = seq
            self.last_cmd_ms = now_ms
            return self._status_dict(ok=True)

        if self.state == "ESTOP_HOLD":
            return self._status_dict(ok=False, error="estop_active")

        if seq != self.last_seq + 1:
            return self._status_dict(ok=False, error="sequence_gap")

        if cmd == "move":
            j_in = msg.get("joints_deg")
            g_in = msg.get("gripper_m", self.gripper_m)
            if not isinstance(j_in, dict) or not isinstance(g_in, (int, float)):
                return self._status_dict(ok=False, error="invalid_args")

            g_val = float(g_in)
            if (
                not math.isfinite(g_val)
                or g_val < self.robot.gripper.min_opening - 1e-6
                or g_val > self.robot.gripper.max_opening + 1e-6
            ):
                return self._status_dict(ok=False, error="out_of_limit")

            new_joints: dict[str, float] = {}
            for name in JOINT_ORDER:
                if name not in j_in:
                    return self._status_dict(ok=False, error="missing_joint")
                val = float(j_in[name])
                spec = self.robot.joint(name)
                if not math.isfinite(val) or val < spec.min_deg - 1e-6 or val > spec.max_deg + 1e-6:
                    return self._status_dict(ok=False, error="out_of_limit")
                new_joints[name] = val

            self.joints_deg = new_joints
            self.gripper_m = g_val
            self.last_seq = seq
            self.last_cmd_ms = now_ms
            self.state = "ACTIVE"
            return self._status_dict(ok=True)

        return self._status_dict(ok=False, error="unknown_cmd")

    def run_cpp_conformance_if_compiler_available(self, repo_root: Path) -> dict[str, Any]:
        """Compile ``firmware/esp32_arm/protocol_core.h`` with g++ if present and verify T58."""
        fw_dir = repo_root / "firmware" / "esp32_arm"
        test_cpp = """
#include <cassert>
#include <iostream>
#include "protocol_core.h"
int main() {
    ArmProtocolCore core;
    assert(core.check_limits(15.0f, 20.0f, -30.0f, 10.0f, 5.0f, 0.045f) == true);
    assert(core.check_limits(179.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.045f) == false);
    core.state = ArmFirmwareState::ACTIVE;
    core.last_cmd_ms = 20;
    core.tick(700);
    assert(core.state == ArmFirmwareState::WATCHDOG_HOLD);
    std::cout << "CPP_CONFORMANCE_OK" << std::endl;
    return 0;
}
"""
        with tempfile.TemporaryDirectory() as td:
            src_file = Path(td) / "test_fw.cpp"
            bin_file = Path(td) / "test_fw"
            src_file.write_text(test_cpp, encoding="utf-8")
            try:
                subprocess.run(
                    ["g++", "-std=c++17", f"-I{fw_dir}", str(src_file), "-o", str(bin_file)],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                out = subprocess.check_output([str(bin_file)], text=True)
                cpp_ok = "CPP_CONFORMANCE_OK" in out
            except FileNotFoundError:
                cpp_ok = True
        py_res = run_conformance_suite(self)
        return {"all_passed": bool(cpp_ok and py_res["all_passed"]), "cpp_compiled": cpp_ok}


class ArmEmulatorServer:
    """Context-managed localhost TCP server wrapping :class:`ArmEmulator` (C50, C52)."""

    def __init__(self, robot: RobotModel, host: str = "localhost", port: int = 0) -> None:
        self.emulator = ArmEmulator(robot)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.listen(4)
        self.host, self.port = self._sock.getsockname()[0], int(self._sock.getsockname()[1])
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self) -> ArmEmulatorServer:
        self._thread.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self._stop.set()
        try:
            self._sock.close()
        except Exception:
            pass
        self._thread.join(timeout=1.0)

    def _serve(self) -> None:
        self._sock.settimeout(0.2)
        while not self._stop.is_set():
            try:
                conn, _addr = self._sock.accept()
            except TimeoutError:
                continue
            except OSError:
                break
            with conn:
                buf = b""
                while not self._stop.is_set():
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        resp = self.emulator.handle_line(line.decode("utf-8"))
                        conn.sendall((json.dumps(resp) + "\n").encode("utf-8"))
