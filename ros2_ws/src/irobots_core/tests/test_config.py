"""Tests for the configuration loader (PLAN:irobots-v2:C11, C67-F4).

The legacy repo had two copies of ``config.py`` with hardcoded Tailscale IPs
and a single magic number that meant different things in different files.
These tests are the guard rail on the replacement: one source of truth, and
no endpoint or path baked into source.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from irobots_core.config import ConfigError, env_or, load_yaml, scalar_from

#: Matches an IPv4 address or a bare host:port in source.
_ENDPOINT_RE = re.compile(r"\b(?:\d{1,3}\.){3}\d{1,3}\b|\b\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}:\d+")


def test_a_missing_config_fails_loudly() -> None:
    """T11: a missing file must never be read as "no settings"."""
    with pytest.raises(ConfigError):
        load_yaml("config/nope.yaml")


def test_a_scalar_without_a_value_fails() -> None:
    with pytest.raises(ConfigError):
        scalar_from({"provenance": "design"}, "thing")


def test_a_scalar_with_a_bad_provenance_fails() -> None:
    with pytest.raises(ConfigError):
        scalar_from({"value": 1.0, "provenance": "vibes"}, "thing")


def test_env_or_treats_an_empty_string_as_unset() -> None:
    import os

    os.environ["IROBOTS_TEST_EMPTY"] = "   "
    try:
        assert env_or("IROBOTS_TEST_EMPTY", "fallback") == "fallback"
        os.environ["IROBOTS_TEST_SET"] = "value"
        assert env_or("IROBOTS_TEST_SET", "fallback") == "value"
    finally:
        os.environ.pop("IROBOTS_TEST_EMPTY", None)
        os.environ.pop("IROBOTS_TEST_SET", None)


def test_no_endpoint_is_hardcoded_in_the_core(repo_root: Path) -> None:
    """T12: every endpoint in irobots_core and irobots_nodes comes from config or the environment."""
    search_roots = (
        repo_root / "ros2_ws" / "src" / "irobots_core" / "irobots_core",
        repo_root / "ros2_ws" / "src" / "irobots_nodes" / "irobots_nodes",
    )
    offenders: list[str] = []
    for root in search_roots:
        for path in sorted(root.rglob("*.py")):
            for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                if _ENDPOINT_RE.search(line):
                    offenders.append(f"{path.relative_to(repo_root)}:{lineno}: {line.strip()}")
    assert not offenders, "hardcoded endpoints in source:\n" + "\n".join(offenders)


def test_the_core_does_not_import_ros(repo_root: Path) -> None:
    """The core must stay importable without a ROS graph, and ROS 2 deletable."""
    core = repo_root / "ros2_ws" / "src" / "irobots_core" / "irobots_core"
    forbidden = ("rclpy", "rosidl", "launch_ros", "geometry_msgs", "sensor_msgs")
    offenders: list[str] = []
    for path in sorted(core.rglob("*.py")):
        text = path.read_text(encoding="utf-8")
        for lineno, line in enumerate(text.splitlines(), 1):
            stripped = line.strip()
            if not stripped.startswith(("import ", "from ")):
                continue
            if any(name in stripped for name in forbidden):
                offenders.append(f"{path.relative_to(repo_root)}:{lineno}: {stripped}")
    assert not offenders, "the core imports ROS:\n" + "\n".join(offenders)
