"""Typed configuration loading for irobots.

The legacy repo had two copies of ``config.py`` (root and
``src/vision-AI/config/``) with hardcoded Tailscale IPs and a single scalar
called ``PIXELS_PER_CM`` that meant different things in different files.
This module replaces that with one loader, one schema, and an explicit
provenance tag on every physical constant.

PLAN:irobots-v2:C11
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Final

import yaml

#: Repo root, resolved from this file's location.
#: ``ros2_ws/src/irobots_core/irobots_core/config.py`` -> repo root is 4 levels up.
REPO_ROOT: Final[Path] = Path(__file__).resolve().parents[4]
CONFIG_DIR: Final[Path] = REPO_ROOT / "config"

#: Valid values for the ``provenance`` field of a physical constant.
PROVENANCE_VALUES: Final[frozenset[str]] = frozenset({"measured", "design", "assumed"})


class ConfigError(ValueError):
    """Raised when a config file is missing, malformed, or contradicts the schema."""


@dataclass(frozen=True, slots=True)
class Provenance:
    """Where a physical constant came from.

    ``measured`` means it was read off the STL meshes in
    ``robotic arm schematics/``. ``design`` means we chose it because the
    SolidWorks assembly is an opaque binary with no recoverable joint frames.
    ``assumed`` means it is a guess that must be replaced by measurement on the
    real arm.
    """

    value: float
    provenance: str
    note: str = ""

    def __post_init__(self) -> None:
        if self.provenance not in PROVENANCE_VALUES:
            raise ConfigError(
                f"invalid provenance {self.provenance!r}; expected one of {sorted(PROVENANCE_VALUES)}"
            )

    def __float__(self) -> float:
        return float(self.value)


@dataclass(frozen=True, slots=True)
class Scalar:
    """A float with a provenance tag."""

    value: float
    provenance: str = "design"
    note: str = ""

    def __post_init__(self) -> None:
        if self.provenance not in PROVENANCE_VALUES:
            raise ConfigError(
                f"invalid provenance {self.provenance!r} for value {self.value!r}"
            )

    def __float__(self) -> float:
        return float(self.value)


def load_yaml(path: str | Path) -> dict[str, Any]:
    """Load a YAML file into a dict, with loud errors.

    Raises ``ConfigError`` rather than returning an empty dict, so a missing or
    malformed config can never be silently interpreted as "no settings".
    """
    p = Path(path)
    if not p.is_absolute():
        p = REPO_ROOT / p
    if not p.exists():
        raise ConfigError(f"config file not found: {p}")
    with p.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ConfigError(f"config file {p} must contain a mapping at the top level")
    return data


def _require(mapping: dict[str, Any], key: str, where: str) -> Any:
    if key not in mapping:
        raise ConfigError(f"missing required key {key!r} in {where}")
    return mapping[key]


def scalar_from(node: dict[str, Any], where: str) -> Scalar:
    """Build a :class:`Scalar` from a ``{value, provenance, note}`` mapping."""
    value = _require(node, "value", where)
    if not isinstance(value, (int, float)):
        raise ConfigError(f"{where}.value must be a number, got {type(value).__name__}")
    provenance = node.get("provenance", "design")
    if provenance not in PROVENANCE_VALUES:
        raise ConfigError(
            f"{where}.provenance={provenance!r} invalid; expected one of {sorted(PROVENANCE_VALUES)}"
        )
    return Scalar(float(value), provenance, str(node.get("note", "")))


def env_or(name: str, default: str | None = None) -> str | None:
    """Read an environment variable, treating the empty string as unset.

    Every endpoint, model id and API key in this project comes through here or
    through a config file. Nothing is hardcoded in source (PLAN:irobots-v2:T12).
    """
    raw = os.environ.get(name)
    if raw is None:
        return default
    raw = raw.strip()
    return raw if raw else default
