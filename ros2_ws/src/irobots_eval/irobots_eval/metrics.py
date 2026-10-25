"""Privileged and perceived evaluation metrics (PLAN:irobots-v2:C41, T53, I5).

Invariant I5: every metric is explicitly tagged ``privileged`` (computed from
simulator ground truth) or ``perceived`` (computed from what perception
observes). The two are never mixed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from typing import Any, Generic, TypeVar

T = TypeVar("T")


class MetricTag(StrEnum):
    """Whether a metric comes from simulator ground truth or perception (I5)."""

    PRIVILEGED = "privileged"
    PERCEIVED = "perceived"


@dataclass(frozen=True, slots=True)
class TaggedMetric(Generic[T]):
    name: str
    value: T
    tag: MetricTag

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "value": self.value, "tag": self.tag.value}


@dataclass(frozen=True, slots=True)
class EpisodeMetricsRecord:
    episode_id: str
    task_success_privileged: TaggedMetric[bool]
    task_success_perceived: TaggedMetric[bool]
    grasp_success_privileged: TaggedMetric[bool]
    grasp_success_perceived: TaggedMetric[bool]
    steps: int
    wall_time_s: float
    llm_calls: int
    invalid_tool_calls: int
    retries: int
    safety_rejections: int
    violations: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "episode_id": self.episode_id,
            "task_success_privileged": self.task_success_privileged.to_dict(),
            "task_success_perceived": self.task_success_perceived.to_dict(),
            "grasp_success_privileged": self.grasp_success_privileged.to_dict(),
            "grasp_success_perceived": self.grasp_success_perceived.to_dict(),
            "steps": self.steps,
            "wall_time_s": self.wall_time_s,
            "llm_calls": self.llm_calls,
            "invalid_tool_calls": self.invalid_tool_calls,
            "retries": self.retries,
            "safety_rejections": self.safety_rejections,
            "violations": self.violations,
        }


@dataclass(frozen=True, slots=True)
class BinPlacementGrade:
    task_success_privileged: TaggedMetric[bool]
    task_success_perceived: TaggedMetric[bool]
    radial_error_m: float
    resting_on_surface: bool


def grade_l2_bin_placement(
    *,
    object_xyz_privileged: tuple[float, float, float],
    bin_xy_privileged: tuple[float, float],
    bin_radius_m: float,
    gripper_xyz: tuple[float, float, float],
    gripper_opening: float,
    perceived_released_over_bin: bool,
    object_half_height_m: float = 0.020,
) -> BinPlacementGrade:
    """Grade L2 placement on privileged final world pose vs perceived release (T53, I5).

    Distinguishes:
    1. In-bin and resting on the surface (`z <= half_height + 0.008` and `r <= bin_radius`).
    2. Near-bin (`r > bin_radius`).
    3. Held in air (`z > half_height + 0.008` or gripper still closed on the object).
    """
    ox, oy, oz = object_xyz_privileged
    bx, by = bin_xy_privileged
    radial_err = math.hypot(ox - bx, oy - by)
    resting = abs(oz - object_half_height_m) <= 0.008 and gripper_opening > 0.045
    in_bin_priv = bool(radial_err <= bin_radius_m and resting)
    return BinPlacementGrade(
        task_success_privileged=TaggedMetric(
            "task_success", in_bin_priv, MetricTag.PRIVILEGED
        ),
        task_success_perceived=TaggedMetric(
            "task_success", bool(perceived_released_over_bin), MetricTag.PERCEIVED
        ),
        radial_error_m=radial_err,
        resting_on_surface=resting,
    )
