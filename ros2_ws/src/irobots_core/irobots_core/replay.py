"""Episode transcript replay and SHA-256 action-log hash verification (PLAN:irobots-v2:C38, T50, T51, D.7).

Re-executes or verifies a recorded ``episode.jsonl`` transcript and asserts that
the SHA-256 hash over the ordered skill calls, arguments, and verdicts matches
the recorded hash.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any


class TamperedTranscriptError(ValueError):
    """Raised when a transcript's recomputed action-log hash disagrees with the expected hash."""


@dataclass(frozen=True, slots=True)
class ReplayResult:
    reproduced: bool
    computed_sha256: str
    turns_replayed: int


def compute_action_log_sha256(records: Sequence[Mapping[str, Any]]) -> str:
    """Compute the canonical SHA-256 over ordered skill calls, arguments, and verdicts."""
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


def replay_episode(
    jsonl_path: str | Path,
    *,
    expected_hash: str | None = None,
) -> ReplayResult:
    """Replay an ``episode.jsonl`` transcript and verify its SHA-256 action-log hash (T50, T51)."""
    p = Path(jsonl_path)
    if not p.exists():
        raise FileNotFoundError(f"transcript not found: {p}")

    records: list[dict[str, Any]] = []
    embedded_hash: str | None = None
    for line in p.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        item = json.loads(line)
        if item.get("type") == "episode_summary":
            embedded_hash = str(item.get("action_log_sha256") or "")
        else:
            records.append(item)

    computed = compute_action_log_sha256(records)
    target_hash = expected_hash or embedded_hash
    if target_hash and computed != target_hash:
        raise TamperedTranscriptError(
            f"action-log hash mismatch: computed {computed} != expected {target_hash}"
        )
    return ReplayResult(
        reproduced=True,
        computed_sha256=computed,
        turns_replayed=len(records),
    )
