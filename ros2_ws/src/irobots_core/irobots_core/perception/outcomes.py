"""Typed outcome predicates for closed-loop verification (PLAN:irobots-v2:C17, C18, C67-F3).

The verification edge compares the scene before and after a skill execution.
Because the table in ``config/scene.yaml`` has an explicit 20 mm rim (C18),
"the object was lifted" is cleanly distinguishable from "the object was knocked
away".
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from typing import Any


class OutcomeType(StrEnum):
    """Canonical verification outcomes reported back to the planner and worker."""

    OBJECT_LIFTED = "object_lifted"
    OBJECT_MOVED_WITH_GRIPPER = "object_moved_with_gripper"
    OBJECT_KNOCKED_AWAY = "object_knocked_away"
    NO_TARGET = "no_target"
    GRIPPER_AT_POSE = "gripper_at_pose"
    TABLE_HIT = "table_hit"
    SAFETY_REJECTED = "safety_rejected"
    CAMERA_UNAVAILABLE = "camera_unavailable"


@dataclass(frozen=True, slots=True)
class SceneObservation:
    """Snapshot of the arm and object poses before or after a skill."""

    gripper_xyz: tuple[float, float, float]
    gripper_opening: float
    objects: Mapping[str, tuple[float, float, float]]


@dataclass(frozen=True, slots=True)
class OutcomeEvaluation:
    """Structured outcome evaluation after one skill execution."""

    primary: OutcomeType
    predicates: Mapping[str, bool]
    detail: str = ""


def is_inside_table_rim(
    xy_m: tuple[float, float],
    scene_cfg: Mapping[str, Any],
) -> bool:
    """Return True if ``xy_m`` lies inside the inner boundary of the table rim (C18)."""
    table = scene_cfg["table"]
    sx, sy = (float(c) for c in table["size_xy"])
    px, py = (float(c) for c in table.get("position", [0.0, 0.12]))
    rim = table.get("rim") or {}
    rim_t = float(rim.get("thickness", 0.01)) if rim.get("enabled", True) else 0.0
    hx = sx / 2.0 - rim_t
    hy = sy / 2.0 - rim_t
    return (px - hx <= float(xy_m[0]) <= px + hx) and (py - hy <= float(xy_m[1]) <= py + hy)


def gripper_at_pose(
    after: SceneObservation,
    desired_xyz: tuple[float, float, float],
    tolerance_m: float = 0.015,
) -> bool:
    """Return True if the gripper reached ``desired_xyz`` within ``tolerance_m``."""
    return math.dist(after.gripper_xyz, desired_xyz) <= tolerance_m


def table_hit(
    after: SceneObservation,
    table_z: float = 0.0,
    clearance_m: float = 0.001,
) -> bool:
    """Return True if the gripper contacted or penetrated the table surface."""
    return after.gripper_xyz[2] <= table_z + clearance_m


def no_target(
    after: SceneObservation,
    target_id: str | None,
) -> bool:
    """Return True if a required ``target_id`` is missing from the observed scene."""
    if not target_id:
        return False
    return target_id not in after.objects


def object_lifted(
    before: SceneObservation,
    after: SceneObservation,
    target_id: str,
    *,
    min_gain_m: float = 0.015,
    max_xy_slip_m: float = 0.035,
) -> bool:
    """Return True if ``target_id`` rose by at least ``min_gain_m`` and stayed with the gripper."""
    if target_id not in before.objects or target_id not in after.objects:
        return False
    b_pos = before.objects[target_id]
    a_pos = after.objects[target_id]
    gain_z = a_pos[2] - b_pos[2]
    xy_slip = math.hypot(a_pos[0] - after.gripper_xyz[0], a_pos[1] - after.gripper_xyz[1])
    return gain_z >= min_gain_m and xy_slip <= max_xy_slip_m


def object_moved_with_gripper(
    before: SceneObservation,
    after: SceneObservation,
    target_id: str,
    *,
    max_xy_slip_m: float = 0.035,
) -> bool:
    """Return True if ``target_id`` remained within ``max_xy_slip_m`` of the gripper after motion."""
    if target_id not in before.objects or target_id not in after.objects:
        return False
    a_pos = after.objects[target_id]
    xy_slip = math.hypot(a_pos[0] - after.gripper_xyz[0], a_pos[1] - after.gripper_xyz[1])
    return xy_slip <= max_xy_slip_m and after.gripper_opening < 0.065


def object_knocked_away(
    before: SceneObservation,
    after: SceneObservation,
    target_id: str,
    scene_cfg: Mapping[str, Any],
    *,
    displacement_threshold_m: float = 0.045,
) -> bool:
    """Return True if ``target_id`` was displaced outside the rim or far from the gripper on the table."""
    if target_id not in before.objects or target_id not in after.objects:
        return False
    b_pos = before.objects[target_id]
    a_pos = after.objects[target_id]
    if not is_inside_table_rim((a_pos[0], a_pos[1]), scene_cfg):
        return True
    if object_lifted(before, after, target_id):
        return False
    disp_xy = math.hypot(a_pos[0] - b_pos[0], a_pos[1] - b_pos[1])
    dist_to_gripper = math.hypot(a_pos[0] - after.gripper_xyz[0], a_pos[1] - after.gripper_xyz[1])
    return disp_xy >= displacement_threshold_m and dist_to_gripper > 0.035


def evaluate_outcome(
    before: SceneObservation,
    after: SceneObservation,
    target_id: str | None,
    desired_gripper_xyz: tuple[float, float, float],
    scene_cfg: Mapping[str, Any],
) -> OutcomeEvaluation:
    """Compute all outcome predicates and select the primary outcome."""
    table_z = float(scene_cfg.get("table", {}).get("height", 0.0))
    p_hit = table_hit(after, table_z=table_z)
    p_no_target = no_target(after, target_id)
    p_at_pose = gripper_at_pose(after, desired_gripper_xyz)
    p_lifted = object_lifted(before, after, target_id) if target_id else False
    p_moved = object_moved_with_gripper(before, after, target_id) if target_id else False
    p_knocked = object_knocked_away(before, after, target_id, scene_cfg) if target_id else False

    preds = {
        "table_hit": p_hit,
        "no_target": p_no_target,
        "object_knocked_away": p_knocked,
        "object_lifted": p_lifted,
        "object_moved_with_gripper": p_moved,
        "gripper_at_pose": p_at_pose,
    }

    if p_hit:
        primary = OutcomeType.TABLE_HIT
    elif p_no_target:
        primary = OutcomeType.NO_TARGET
    elif p_knocked:
        primary = OutcomeType.OBJECT_KNOCKED_AWAY
    elif p_lifted:
        primary = OutcomeType.OBJECT_LIFTED
    elif p_moved:
        primary = OutcomeType.OBJECT_MOVED_WITH_GRIPPER
    else:
        primary = OutcomeType.GRIPPER_AT_POSE

    return OutcomeEvaluation(primary=primary, predicates=preds, detail=primary.value)
