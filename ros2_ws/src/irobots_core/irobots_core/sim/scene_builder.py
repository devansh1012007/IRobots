"""Deterministic seeded scene builder (PLAN:irobots-v2:C22).

The same 30 scene seeds are reused across every experimental condition (RQ0,
RQ1, RQ2, RQ3) so that comparisons are strictly paired (McNemar's test).
Every random draw goes through a seeded ``numpy.random.Generator`` threaded
through the call chain — never global ``random`` or bare ``np.random.rand``.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Final

import numpy as np

from ..config import ConfigError, load_yaml
from ..robot_model import RobotModel

#: The canonical 30 scene seeds shared across all paired evaluation conditions.
SHARED_EVAL_SEEDS: Final[tuple[int, ...]] = tuple(range(30))


@dataclass(frozen=True, slots=True)
class SpawnedObject:
    """One task object, distractor, or marked bin region in a seeded scene."""

    name: str
    label: str
    size: tuple[float, float, float]
    position: tuple[float, float, float]
    yaw_rad: float
    mass: float
    friction: float
    rgba: str
    is_bin: bool = False
    provenance: str = "design"


@dataclass(frozen=True, slots=True)
class TaskSpec:
    """Typed task definition loaded from ``config/tasks/*.yaml``."""

    name: str
    level: str
    instruction: str
    target_object: str
    target_bin: str | None
    goal_type: str
    turn_budget: int
    episode_deadline_s: float
    objects: Sequence[Mapping[str, Any]]
    distractors: Sequence[Mapping[str, Any]]
    bins: Sequence[Mapping[str, Any]]
    provenance: str = "design"


@dataclass(frozen=True, slots=True)
class SeededScene:
    """A deterministically sampled scene for a given seed and task."""

    seed: int
    task_name: str
    level: str
    instruction: str
    target_object: str
    target_bin: str | None
    objects: tuple[SpawnedObject, ...]

    def to_sim_objects(self) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        for obj in self.objects:
            out.append(
                {
                    "name": obj.name,
                    "label": obj.label,
                    "size": list(obj.size),
                    "pos": list(obj.position),
                    "yaw_rad": obj.yaw_rad,
                    "mass": obj.mass,
                    "friction": obj.friction,
                    "rgba": obj.rgba,
                    "is_bin": obj.is_bin,
                }
            )
        return out

    def digest(self) -> str:
        """SHA-256 digest of the scene specification for reproducibility checks."""
        payload = json.dumps(
            {
                "seed": self.seed,
                "task_name": self.task_name,
                "objects": [
                    {
                        "name": o.name,
                        "label": o.label,
                        "pos": [round(c, 8) for c in o.position],
                        "yaw": round(o.yaw_rad, 8),
                    }
                    for o in self.objects
                ],
            },
            sort_keys=True,
        )
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def load_task_spec(path: str | Path) -> TaskSpec:
    """Load a :class:`TaskSpec` from a YAML file."""
    raw = load_yaml(path)
    t = raw.get("task")
    if not isinstance(t, dict):
        raise ConfigError(f"{path}: missing top-level 'task' mapping")
    return TaskSpec(
        name=str(t["name"]),
        level=str(t["level"]),
        instruction=str(t["instruction"]),
        target_object=str(t["target_object"]),
        target_bin=str(t["target_bin"]) if t.get("target_bin") else None,
        goal_type=str(t.get("goal_type", "lift")),
        turn_budget=int(t.get("turn_budget", 10)),
        episode_deadline_s=float(t.get("episode_deadline_s", 30.0)),
        objects=list(t.get("objects", [])),
        distractors=list(t.get("distractors", [])),
        bins=list(t.get("bins", [])),
        provenance=str(t.get("provenance", "design")),
    )


def _sample_non_overlapping_xy(
    rng: np.random.Generator,
    centre: tuple[float, float],
    radius: float,
    existing: Sequence[tuple[float, float]],
    min_separation_m: float = 0.065,
) -> tuple[float, float, float]:
    for _ in range(200):
        r = radius * math.sqrt(float(rng.random()))
        theta = float(rng.random()) * 2.0 * math.pi
        x = centre[0] + r * math.cos(theta)
        y = centre[1] + r * math.sin(theta)
        if all(math.hypot(x - ex, y - ey) >= min_separation_m for ex, ey in existing):
            yaw = float(rng.random()) * (math.pi / 2.0) - (math.pi / 4.0)
            return (x, y, yaw)
    raise RuntimeError("could not sample non-overlapping object poses in workspace")


def build_seeded_scene(
    robot: RobotModel,
    scene_cfg: Mapping[str, Any],
    task: TaskSpec,
    seed: int,
) -> SeededScene:
    """Deterministically sample a scene from ``seed``."""
    rng = np.random.default_rng(int(seed))
    ws = scene_cfg.get("workspace", {})
    c_raw = ws.get("centre_xy", list(robot.safety.workspace_centre))
    centre = (float(c_raw[0]), float(c_raw[1]))
    radius = float(ws.get("radius", robot.safety.workspace_radius)) * 0.78

    placed_xy: list[tuple[float, float]] = []
    spawned: list[SpawnedObject] = []

    for node in list(task.objects) + list(task.distractors):
        sz = tuple(float(c) for c in node["size"])
        x, y, yaw = _sample_non_overlapping_xy(rng, centre, radius, placed_xy)
        placed_xy.append((x, y))
        spawned.append(
            SpawnedObject(
                name=str(node["name"]),
                label=str(node.get("label", node["name"])),
                size=(sz[0], sz[1], sz[2]),
                position=(x, y, sz[2] / 2.0),
                yaw_rad=yaw,
                mass=float(node.get("mass", 0.04)),
                friction=float(node.get("friction", 0.6)),
                rgba=str(node.get("rgba", "0.85 0.25 0.25 1")),
                is_bin=False,
                provenance=str(node.get("provenance", "design")),
            )
        )

    for bnode in task.bins:
        sz = tuple(float(c) for c in bnode["size"])
        x, y, _yaw = _sample_non_overlapping_xy(
            rng, centre, radius, placed_xy, min_separation_m=0.070
        )
        placed_xy.append((x, y))
        spawned.append(
            SpawnedObject(
                name=str(bnode["name"]),
                label=str(bnode.get("label", bnode["name"])),
                size=(sz[0], sz[1], sz[2]),
                position=(x, y, sz[2] / 2.0),
                yaw_rad=0.0,
                mass=0.10,
                friction=0.8,
                rgba=str(bnode.get("rgba", "0.25 0.78 0.35 1")),
                is_bin=True,
                provenance=str(bnode.get("provenance", "design")),
            )
        )

    return SeededScene(
        seed=int(seed),
        task_name=task.name,
        level=task.level,
        instruction=task.instruction,
        target_object=task.target_object,
        target_bin=task.target_bin,
        objects=tuple(spawned),
    )
