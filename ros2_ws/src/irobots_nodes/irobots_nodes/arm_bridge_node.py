"""ROS 2 <-> TCP NDJSON bridge driving either the ESP32 emulator or the real ESP32 (PLAN:irobots-v2:C52, T59)."""

from __future__ import annotations

import json
import socket
from collections.abc import Mapping
from typing import Any

from irobots_core.config import env_or


class ArmBridgeNode:
    """Thin TCP NDJSON bridge to the arm controller or emulator (C52)."""

    def __init__(
        self,
        *,
        host: str | None = None,
        port: int | None = None,
        timeout_s: float = 2.0,
    ) -> None:
        self.host = host or env_or("ARM_HOST", "localhost") or "localhost"
        self.port = int(port if port is not None else int(env_or("ARM_PORT", "9000") or "9000"))
        self.timeout_s = timeout_s

    def _send_payload(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        line = (json.dumps(dict(payload)) + "\n").encode("utf-8")
        with socket.create_connection((self.host, self.port), timeout=self.timeout_s) as sock:
            sock.sendall(line)
            buf = b""
            while b"\n" not in buf:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
        resp_line = buf.split(b"\n", 1)[0].decode("utf-8")
        res: dict[str, Any] = json.loads(resp_line)
        return res

    def send_joint_command(
        self,
        *,
        seq: int,
        joints_deg: Mapping[str, float],
        gripper_m: float,
    ) -> dict[str, Any]:
        return self._send_payload(
            {
                "cmd": "move",
                "seq": int(seq),
                "joints_deg": {str(k): float(v) for k, v in joints_deg.items()},
                "gripper_m": float(gripper_m),
            }
        )

    def send_estop(self, *, seq: int) -> dict[str, Any]:
        return self._send_payload({"cmd": "estop", "seq": int(seq)})
