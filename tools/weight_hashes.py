#!/usr/bin/env python
"""Compute and record SHA-256 hashes of all model weights (PLAN:irobots-v2:C62, T63)."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        while chunk := fh.read(65536):
            h.update(chunk)
    return h.hexdigest()


def collect_weight_hashes(repo_root: Path = REPO_ROOT) -> dict[str, str]:
    hashes: dict[str, str] = {}
    candidates = list(repo_root.glob("*.pt")) + list((repo_root / "weights").glob("*.pt"))
    for p in sorted(candidates):
        if p.is_file():
            hashes[str(p.relative_to(repo_root))] = sha256_file(p)
    return hashes


def main() -> int:
    hashes = collect_weight_hashes(REPO_ROOT)
    print(json.dumps(hashes, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
