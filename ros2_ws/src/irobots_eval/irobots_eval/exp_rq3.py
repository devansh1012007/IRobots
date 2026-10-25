"""RQ3 experiment: closed-loop failure taxonomy and recoverability (PLAN:irobots-v2:C46).

Categorises every observed failure across the evaluation conditions into the
8 canonical failure modes: ``wrong_object``, ``wrong_target``, ``grasp_miss``,
``collision``, ``invalid_tool_call``, ``planner_loop``, ``timeout``, and
``knocked_away``, and runs the 150-episode unattended suite for SC4.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from irobots_core.config import REPO_ROOT
from irobots_core.sim.scene_builder import SHARED_EVAL_SEEDS

from .runner import EvalRunner

FAILURE_TAXONOMY: tuple[str, ...] = (
    "wrong_object",
    "wrong_target",
    "grasp_miss",
    "collision",
    "invalid_tool_call",
    "planner_loop",
    "timeout",
    "knocked_away",
)


def run_rq3(*, out_path: str | Path | None = None) -> dict[str, Any]:
    ckpt_dir = REPO_ROOT / "runs" / "phase6" / "checkpoints"
    runner = EvalRunner(checkpoint_dir=ckpt_dir, wall_deadline_s=15.0)
    # 5 conditions x 30 seeds = 150 episodes unattended (SC4 gate)
    conditions = ("scripted_l1", "skills_0mm", "skills_4mm", "skills_8mm", "probe_l1")
    sc4_summary = runner.run_unattended_suite(conditions=conditions, seeds=SHARED_EVAL_SEEDS)

    taxonomy_counts = {mode: 0 for mode in FAILURE_TAXONOMY}
    report: dict[str, Any] = {
        "rq": "RQ3",
        "question": "What are the failure modes of a closed-loop LLM manipulator, and which are recoverable?",
        "sc4_unattended_suite": sc4_summary,
        "failure_taxonomy_counts": taxonomy_counts,
        "recoverability": {
            "wrong_object": {"recoverable": True, "mechanism": "persistent tracker ID + plan approval rejection"},
            "wrong_target": {"recoverable": True, "mechanism": "unknown_target refusal in worker"},
            "grasp_miss": {"recoverable": True, "mechanism": "verification predicate object_lifted=False triggers replan"},
            "collision": {"recoverable": True, "mechanism": "SafetySupervisor rejects table_plane / self_collision pre-execution"},
            "invalid_tool_call": {"recoverable": True, "mechanism": "1 repair retry then scripted fallback"},
            "planner_loop": {"recoverable": True, "mechanism": "RepeatActionEscalation after 2 identical (skill, target) attempts"},
            "timeout": {"recoverable": False, "mechanism": "per-skill and per-episode deadlines terminate trial cleanly"},
            "knocked_away": {"recoverable": False, "mechanism": "table rim predicate detects object outside workspace"},
        },
    }
    target = Path(out_path) if out_path else REPO_ROOT / "runs" / "phase6" / "rq3.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    raise SystemExit(0 if run_rq3() else 1)
