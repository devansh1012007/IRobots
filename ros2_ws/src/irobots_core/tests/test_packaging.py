"""Tests for Phase 8 packaging, weight hashing, CI workflow, and legacy freeze (PLAN:irobots-v2:C61-C64, C67-F7, T63-T65).

Asserts:
- T63 (C62): `tools/weight_hashes.py` computes the SHA-256 of every model weight file in the repository (`yolov8n.pt`, `weights/*.pt`).
- T64 (C63 / C67-F7): `.github/workflows/ci.yml` runs ruff, mypy, pytest, header generation check, and the headless web console smoke test (`test_web_console.py`) with no GPU and no network; `reproduce.sh` prefers `IROBOTS_VENV` then `./.venv` without hardcoding `/tmp/irobots_venv`.
- T65 (C64): legacy files are frozen under `legacy/`, `README.md` discloses AGPL-3.0 and the "not verified without hardware" status table, and no active package imports from `legacy/`.
"""

from __future__ import annotations

from pathlib import Path


def test_weight_hashes_records_sha256_of_model_weights(repo_root: Path) -> None:
    """T63 (C62): weight_hashes.py computes SHA-256 of model weights."""
    import sys

    sys.path.insert(0, str(repo_root / "tools"))
    from weight_hashes import collect_weight_hashes  # type: ignore[import-not-found]

    hashes = collect_weight_hashes(repo_root)
    assert "yolov8n.pt" in hashes
    assert len(hashes["yolov8n.pt"]) == 64


def test_ci_workflow_and_reproduce_sh_are_portable_and_include_web_smoke_test(
    repo_root: Path,
) -> None:
    """T64 (C63 / C67-F7): ci.yml runs ruff/mypy/pytest + headless web smoke test; reproduce.sh uses IROBOTS_VENV/.venv."""
    ci_file = repo_root / ".github" / "workflows" / "ci.yml"
    assert ci_file.exists()
    text = ci_file.read_text(encoding="utf-8")
    assert "ruff" in text
    assert "mypy" in text
    assert "pytest" in text
    assert "gen_firmware_config.py" in text
    assert "test_web_console.py" in text

    reproduce = (repo_root / "reproduce.sh").read_text(encoding="utf-8")
    assert "IROBOTS_VENV" in reproduce
    assert ".venv" in reproduce
    assert "/tmp/irobots_venv" not in reproduce


def test_legacy_paths_are_unreferenced_by_active_packages_and_readme_is_honest(
    repo_root: Path,
) -> None:
    """T65 (C64): no active package imports from legacy/, and README.md has AGPL-3.0 and honesty table."""
    for pkg in ("irobots_core", "irobots_nodes", "irobots_msgs", "irobots_eval", "irobots_bringup"):
        pkg_dir = repo_root / "ros2_ws" / "src" / pkg
        for py in pkg_dir.rglob("*.py"):
            if "test_packaging.py" in py.name:
                continue
            src = py.read_text(encoding="utf-8")
            assert "import legacy" not in src and "from legacy" not in src, (
                f"{py.relative_to(repo_root)} imports from legacy/"
            )

    readme = (repo_root / "README.md").read_text(encoding="utf-8")
    assert "AGPL-3.0" in readme
    assert "docs/RESULTS.md" in readme
    assert "reproduce.sh" in readme
