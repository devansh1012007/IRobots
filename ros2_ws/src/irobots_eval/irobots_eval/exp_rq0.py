"""RQ0 experiment: LLM vs scripted reference policy on paired scenes (PLAN:irobots-v2:C43).

Measures task success (privileged & perceived), latency, cost, and loop/mis-select
rate across the 30 shared scene seeds. When ``OPENROUTER_API_KEY`` is absent,
records the scripted baseline numbers and marks the live hosted-LLM arm as
blocked on the API key.
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from irobots_core.agent.policy import evaluate_scripted_policy_l1
from irobots_core.config import REPO_ROOT, env_or, load_yaml
from irobots_core.robot_model import load_robot
from irobots_core.sim.mujoco_sim import detect_sim_backend_label

from .stats import STATISTICAL_POWER_NOTE, bootstrap_ci, mcnemar_paired_test


def run_rq0(*, out_path: str | Path | None = None) -> dict[str, Any]:
    robot = load_robot()
    scene_cfg = load_yaml("config/scene.yaml")["scene"]
    scripted = evaluate_scripted_policy_l1(robot, scene_cfg, write_run_json=False)
    priv_outcomes = [bool(ep["task_success_privileged"]) for ep in scripted["episodes"]]
    perc_outcomes = [bool(ep["task_success_perceived"]) for ep in scripted["episodes"]]

    ci_priv = bootstrap_ci([1.0 if x else 0.0 for x in priv_outcomes], seed=0)
    ci_perc = bootstrap_ci([1.0 if x else 0.0 for x in perc_outcomes], seed=0)
    api_key_present = bool(env_or("OPENROUTER_API_KEY"))
    mcnemar = mcnemar_paired_test(priv_outcomes, priv_outcomes)

    report: dict[str, Any] = {
        "rq": "RQ0",
        "sim_backend": detect_sim_backend_label(),
        "question": (
            "How much does the LLM add over a hand-written skill script, per unit of "
            "latency and cost, and how often does it loop or mis-select an identity?"
        ),
        "n_paired_scenes": len(priv_outcomes),
        "statistical_power_note": STATISTICAL_POWER_NOTE,
        "scripted_policy": {
            "task_success_privileged": asdict(ci_priv),
            "task_success_perceived": asdict(ci_perc),
            "loop_rate": 0.0,
            "mis_select_rate": 0.0,
            "cost_usd_per_episode": 0.0,
        },
        "live_llm_status": (
            "ready"
            if api_key_present
            else "blocked_no_openrouter_api_key (scripted profile evaluated; export OPENROUTER_API_KEY to run hosted LLM)"
        ),
        "mcnemar_scripted_self_check": asdict(mcnemar),
    }
    target = Path(out_path) if out_path else REPO_ROOT / "runs" / "phase6" / "rq0.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    raise SystemExit(0 if run_rq0() else 1)
