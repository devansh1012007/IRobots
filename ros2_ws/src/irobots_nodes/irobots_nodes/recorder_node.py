"""Structured JSONL episode recorder and action-log hash verifier (PLAN:irobots-v2:C27, T35, D.7).

Writes one JSON line per turn with simulation time, scene digest, skill call,
safety verdict, and outcome, and computes the deterministic SHA-256 action-log
hash used by ``--replay``.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any


class RecorderNode:
    """Per-episode structured JSONL recorder (C27)."""

    def __init__(self, jsonl_path: str | Path) -> None:
        self.jsonl_path = Path(jsonl_path)
        self.jsonl_path.parent.mkdir(parents=True, exist_ok=True)
        self._records: list[dict[str, Any]] = []

    def record_turn(
        self,
        *,
        turn_index: int,
        sim_time_s: float,
        scene_digest: str,
        skill: str,
        target_id: str | None,
        safety_approved: bool,
        outcome: str,
        extra: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        entry: dict[str, Any] = {
            "turn_index": int(turn_index),
            "sim_time_s": round(float(sim_time_s), 6),
            "scene_digest": str(scene_digest),
            "skill": str(skill),
            "target_id": target_id or "",
            "safety_approved": bool(safety_approved),
            "outcome": str(outcome),
        }
        if extra:
            entry.update(dict(extra))
        self._records.append(entry)
        with self.jsonl_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(entry, sort_keys=True) + "\n")
        return entry

    def action_log_hash(self) -> str:
        return self.compute_action_log_hash(self._records)

    @staticmethod
    def compute_action_log_hash(records: Sequence[Mapping[str, Any]]) -> str:
        """Compute the canonical SHA-256 hash over ordered skill calls, args, and verdicts (D.7)."""
        canonical = [
            {
                "turn_index": int(r["turn_index"]),
                "skill": str(r["skill"]),
                "target_id": str(r.get("target_id", "")),
                "safety_approved": bool(r["safety_approved"]),
                "outcome": str(r["outcome"]),
            }
            for r in records
        ]
        blob = json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(blob).hexdigest()

    @staticmethod
    def load_jsonl(path: str | Path) -> list[dict[str, Any]]:
        p = Path(path)
        if not p.exists():
            return []
        return [
            json.loads(line)
            for line in p.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
