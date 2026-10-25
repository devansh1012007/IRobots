"""Minimal technical Web UI server for the IRobots v2 control console (PLAN:irobots-v2:C66).

Serves the gradient-free deep-black/grey operator control interface
(``web/irobots_control_console.html``) and exposes JSON API endpoints backed by
``irobots_core`` (`GET /api/state`, `POST /api/camera_source`, `POST /api/step`,
`POST /api/estop`, `POST /api/reset`).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from irobots_core.config import REPO_ROOT, ConfigError, load_yaml
from irobots_core.estop import EStopController
from irobots_core.kinematics import forward_kinematics
from irobots_core.perception.camera_source import CameraSourceConfig, CameraSourceMode
from irobots_core.robot_model import load_robot
from irobots_core.safety import SafetySupervisor


class WebConsoleBackend:
    """Pure-Python state & API handler backing the Web Control Console."""

    def __init__(self, repo_root: Path = REPO_ROOT) -> None:
        self.repo_root = repo_root
        self.html_path = repo_root / "web" / "irobots_control_console.html"
        self.robot = load_robot()
        self.scene_cfg = load_yaml("config/scene.yaml")["scene"]
        self.perception_cfg = load_yaml("config/perception.yaml")["perception"]
        self.camera_source_cfg = CameraSourceConfig.from_configs(
            self.perception_cfg, self.scene_cfg
        )
        self.estop = EStopController()
        self.supervisor = SafetySupervisor(self.robot)
        self.pose: dict[str, float] = {k: float(v) for k, v in self.robot.home_pose.items()}

    def load_html(self) -> str:
        return self.html_path.read_text(encoding="utf-8")

    def set_camera_source(
        self, source: str, *, calibration_file: str | Path | None = None
    ) -> dict[str, Any]:
        try:
            self.camera_source_cfg = CameraSourceConfig.from_configs(
                self.perception_cfg,
                self.scene_cfg,
                source_override=source.strip().lower(),
                calibration_file_override=calibration_file,
            )
            return {"ok": True, "camera_source": self.camera_source_cfg.source.value}
        except (ConfigError, ValueError) as exc:
            return {
                "ok": False,
                "error": str(exc),
                "camera_source": self.camera_source_cfg.source.value,
            }

    def get_state_snapshot(self) -> dict[str, Any]:
        fk = forward_kinematics(self.robot, self.pose)
        return {
            "robot_name": self.robot.name,
            "camera_source": self.camera_source_cfg.source.value,
            "estopped": self.estop.triggered,
            "safety_rejections": self.supervisor.rejected_infeasible,
            "joints_deg": {
                name: round(math.degrees(self.pose[name]), 2)
                for name in self.robot.joints
            },
            "gripper_opening_m": round(float(self.pose.get("gripper", 0.045)), 4),
            "tcp_xyz_m": [round(float(c), 4) for c in fk.position],
        }
