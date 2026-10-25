"""Operator Rich TUI node (PLAN:irobots-v2:C37, T48, T49, D11).

Displays the annotated scene status, tracked-object table, plan tree, current
skill progress, arm state, and safety banner. Supports modes ``plan``
(approve), ``run``, ``step``, and ``replay``, and keys ``p s e r q``.
"""

from __future__ import annotations

from collections.abc import Mapping
from enum import StrEnum
from typing import Any

from irobots_core.estop import EStopController
from irobots_core.robot_model import RobotModel


class TUIMode(StrEnum):
    """Operating mode of the TUI (D11: plan-then-autonomous)."""

    PLAN = "plan"
    RUN = "run"
    STEP = "step"
    REPLAY = "replay"
    QUIT = "quit"


class TUINode:
    """Interactive operator TUI controller supporting `p s e r q` key bindings (C37)."""

    def __init__(self, robot: RobotModel, scene_cfg: Mapping[str, Any]) -> None:
        self.robot = robot
        self.scene_cfg = scene_cfg
        self.mode: TUIMode = TUIMode.PLAN
        self.plan_approved: bool = False
        self.estop = EStopController()
        self.tracked_objects: list[dict[str, Any]] = []
        self.plan_steps: list[str] = ["move_above(red_block_1)", "grasp(red_block_1)", "lift(red_block_1)"]
        self.current_skill: str = "idle"

    def handle_key(self, key: str) -> TUIMode:
        k = key.strip().lower()
        if k == "p":
            self.plan_approved = True
            self.mode = TUIMode.RUN
        elif k == "s":
            self.mode = TUIMode.STEP
        elif k == "e":
            self.estop.trigger(self.robot.home_pose)
        elif k == "r":
            self.mode = TUIMode.REPLAY
        elif k == "q":
            self.mode = TUIMode.QUIT
        return self.mode

    def render_text(self) -> str:
        """Render the TUI state to a string (using rich Console when needed)."""
        from rich.console import Console
        from rich.panel import Panel
        from rich.table import Table

        console = Console(record=True, width=90)
        safety_status = "E-STOP ACTIVE (HOLDING POSE)" if self.estop.triggered else "SAFETY OK"
        console.print(Panel(f"MODE: {self.mode.value.upper()} | {safety_status}", title="SAFETY & MODE"))

        obj_table = Table(title="TRACKED OBJECTS")
        obj_table.add_column("ID")
        obj_table.add_column("Label")
        obj_table.add_column("XY (m)")
        for obj in self.tracked_objects:
            obj_table.add_row(str(obj.get("id")), str(obj.get("label")), str(obj.get("xy")))
        console.print(obj_table)

        plan_str = "\n".join(f"  - {s}" for s in self.plan_steps)
        console.print(Panel(plan_str, title=f"PLAN (approved={self.plan_approved})"))
        return console.export_text()
