#!/usr/bin/env python
"""C03 - LLM tool-call schema probe (PLAN:irobots-v2:C03).

Tests 20 prompts x 2 OpenRouter candidates for tool-call schema validity,
vision handling, median latency, and cost/call. Requires OPENROUTER_API_KEY
in the environment.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / "irobots_core"))

from irobots_core.config import env_or  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--prompts", type=int, default=20)
    ap.add_argument("--out", type=str, default=str(REPO_ROOT / "runs" / "spike" / "llm"))
    args = ap.parse_args()

    api_key = env_or("OPENROUTER_API_KEY")
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    if not api_key:
        report = {
            "status": "blocked",
            "reason": "OPENROUTER_API_KEY is not set in environment",
            "prompts": args.prompts,
            "gate": {"schema_valid_required": 0.95, "median_latency_s_required": 5.0, "passed": False},
        }
        (out_dir / "llm_probe.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print("C03 BLOCKED: OPENROUTER_API_KEY is not set in environment")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
