"""Shared test fixtures.

The core package is pure Python and must be importable without ROS, without
MuJoCo and without the detector. The fixtures here make the workspace packages
importable from a source checkout, so the tests run without an install step or
manual PYTHONPATH.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import pytest

#: The repository root: ros2_ws/src/irobots_core/tests -> repo root.
REPO_ROOT = Path(__file__).resolve().parents[4]
WORKSPACE_PACKAGES: tuple[str, ...] = (
    "irobots_core",
    "irobots_nodes",
    "irobots_msgs",
    "irobots_eval",
    "irobots_bringup",
)

for _pkg in reversed(WORKSPACE_PACKAGES):
    _pkg_src = REPO_ROOT / "ros2_ws" / "src" / _pkg
    if _pkg_src.is_dir() and str(_pkg_src) not in sys.path:
        sys.path.insert(0, str(_pkg_src))


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return REPO_ROOT


@pytest.fixture(scope="session")
def robot() -> Any:
    """The robot model, loaded from config/robot.yaml."""
    from irobots_core.robot_model import load_robot

    return load_robot()


@pytest.fixture(scope="session")
def scene() -> dict[str, Any]:
    """The parsed scene config."""
    from irobots_core.config import load_yaml

    return dict(load_yaml("config/scene.yaml")["scene"])
