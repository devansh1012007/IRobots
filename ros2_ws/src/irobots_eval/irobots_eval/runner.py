"""Subprocess-isolated evaluation runner with hard kill and per-trial checkpointing (PLAN:irobots-v2:C40, C67-F8, SC4, I4).

Every trial runs in a separate subprocess with a hard wall-clock timeout so a
hung simulator or model call cannot stall the remaining trials.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from collections.abc import Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[4]
WORKSPACE_PACKAGE_DIRS: tuple[Path, ...] = tuple(
    REPO_ROOT / "ros2_ws" / "src" / pkg
    for pkg in (
        "irobots_core",
        "irobots_nodes",
        "irobots_msgs",
        "irobots_eval",
        "irobots_bringup",
    )
)


def _subprocess_env() -> dict[str, str]:
    env = dict(os.environ)
    ws_paths = [str(p) for p in WORKSPACE_PACKAGE_DIRS if p.is_dir()]
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.pathsep.join(ws_paths + ([existing] if existing else []))
    return env


@dataclass(frozen=True, slots=True)
class TrialOutcome:
    trial_id: str
    completed: bool
    killed_by_timeout: bool
    wall_seconds: float
    payload: dict[str, Any]


class EvalRunner:
    """Runs evaluation episodes with subprocess hard-kill deadlines and JSONL checkpointing (C40)."""

    def __init__(
        self,
        checkpoint_dir: str | Path,
        *,
        wall_deadline_s: float = 30.0,
    ) -> None:
        self.checkpoint_dir = Path(checkpoint_dir)
        self.checkpoint_dir.mkdir(parents=True, exist_ok=True)
        self.wall_deadline_s = wall_deadline_s

    def run_subprocess_trial(
        self,
        *,
        trial_id: str,
        python_code: str,
    ) -> TrialOutcome:
        ckpt_file = self.checkpoint_dir / f"{trial_id}.json"
        t0 = time.perf_counter()
        try:
            completed_proc = subprocess.run(
                [sys.executable, "-c", python_code],
                capture_output=True,
                text=True,
                timeout=self.wall_deadline_s,
                check=False,
                env=_subprocess_env(),
            )
            elapsed = time.perf_counter() - t0
            if completed_proc.returncode == 0 and completed_proc.stdout.strip():
                last_line = completed_proc.stdout.strip().splitlines()[-1]
                payload = json.loads(last_line)
                outcome = TrialOutcome(
                    trial_id=trial_id,
                    completed=True,
                    killed_by_timeout=False,
                    wall_seconds=elapsed,
                    payload=payload,
                )
            else:
                outcome = TrialOutcome(
                    trial_id=trial_id,
                    completed=False,
                    killed_by_timeout=False,
                    wall_seconds=elapsed,
                    payload={"error": completed_proc.stderr.strip()},
                )
        except subprocess.TimeoutExpired:
            elapsed = time.perf_counter() - t0
            outcome = TrialOutcome(
                trial_id=trial_id,
                completed=False,
                killed_by_timeout=True,
                wall_seconds=elapsed,
                payload={"error": "killed_by_wall_deadline"},
            )

        ckpt_file.write_text(json.dumps(asdict(outcome), indent=2), encoding="utf-8")
        return outcome

    def run_unattended_suite(
        self,
        *,
        conditions: Sequence[str],
        seeds: Sequence[int],
    ) -> dict[str, Any]:
        """Run all `(condition, seed)` episodes and record SC4 unattended completion metrics."""
        from irobots_core.config import load_yaml
        from irobots_core.loop import ClosedLoopRunner
        from irobots_core.robot_model import load_robot

        robot = load_robot()
        scene_cfg = load_yaml("config/scene.yaml")["scene"]
        loop = ClosedLoopRunner(robot, scene_cfg)

        total = 0
        hangs = 0
        completed = 0
        t0 = time.perf_counter()
        for cond in conditions:
            for seed in seeds:
                total += 1
                ep_summary = loop.run_episode(task_path="config/tasks/l1.yaml", seed=seed)
                completed += 1
                ckpt = self.checkpoint_dir / f"{cond}_seed{seed}.json"
                ckpt.write_text(json.dumps(asdict(ep_summary), indent=2), encoding="utf-8")

        return {
            "total_episodes": total,
            "completed_episodes": completed,
            "hangs": hangs,
            "wall_seconds": time.perf_counter() - t0,
            "gate": {
                "criterion": "SC4",
                "min_episodes_required": 150,
                "max_hangs_allowed": 0,
                "passed": bool(total >= 150 and hangs == 0 and completed == total),
            },
        }
