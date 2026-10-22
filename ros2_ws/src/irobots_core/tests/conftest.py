"""Shared test fixtures.

The core package is pure Python and must be importable without ROS, without
MuJoCo and without the detector. The fixtures here make the package importable
from a source checkout, so the tests run without an install step.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

#: The repository root: ros2_ws/src/irobots_core/tests -> repo root.
REPO_ROOT = Path(__file__).resolve().parents[4]
CORE_SRC = REPO_ROOT / "ros2_ws" / "src" / "irobots_core"

if str(CORE_SRC) not in sys.path:
    sys.path.insert(0, str(CORE_SRC))


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return REPO_ROOT


@pytest.fixture(scope="session")
def robot():
    """The robot model, loaded from config/robot.yaml."""
    from irobots_core.robot_model import load_robot

    return load_robot()


@pytest.fixture(scope="session")
def scene() -> dict:
    """The parsed scene config."""
    from irobots_core.config import load_yaml

    return load_yaml("config/scene.yaml")["scene"]
