"""Tests for typed outcome predicates and table-rim boundary checks (PLAN:irobots-v2:C17, C18, T25, T26).

Asserts:
- T25: every outcome predicate (`object_lifted`, `object_moved_with_gripper`,
       `object_knocked_away`, `no_target`, `gripper_at_pose`, `table_hit`)
       returns the expected verdict on synthetic before/after scenes.
- T26: an object resting outside the table rim bounds in `config/scene.yaml`
       is classified as `object_knocked_away` rather than `object_lifted`.
"""

from __future__ import annotations

from irobots_core.perception.outcomes import (
    OutcomeType,
    SceneObservation,
    evaluate_outcome,
    gripper_at_pose,
    is_inside_table_rim,
    no_target,
    object_knocked_away,
    object_lifted,
    object_moved_with_gripper,
    table_hit,
)


def test_every_outcome_predicate_on_synthetic_scenes(scene: dict) -> None:
    """T25: each predicate fires accurately on synthetic before/after scenes."""
    before = SceneObservation(
        gripper_xyz=(0.0, 0.20, 0.06),
        gripper_opening=0.070,
        objects={"red_block_1": (0.0, 0.20, 0.02)},
    )

    # 1. gripper_at_pose
    after_pose = SceneObservation(
        gripper_xyz=(0.0, 0.20, 0.06),
        gripper_opening=0.070,
        objects={"red_block_1": (0.0, 0.20, 0.02)},
    )
    assert gripper_at_pose(after_pose, (0.0, 0.20, 0.06))
    assert not gripper_at_pose(after_pose, (0.08, 0.20, 0.06))

    # 2. object_lifted & object_moved_with_gripper
    after_lift = SceneObservation(
        gripper_xyz=(0.0, 0.20, 0.06),
        gripper_opening=0.040,
        objects={"red_block_1": (0.0, 0.20, 0.055)},
    )
    assert object_lifted(before, after_lift, "red_block_1")
    assert object_moved_with_gripper(before, after_lift, "red_block_1")
    ev_lift = evaluate_outcome(before, after_lift, "red_block_1", (0.0, 0.20, 0.06), scene)
    assert ev_lift.primary is OutcomeType.OBJECT_LIFTED

    # 3. no_target when target_id is absent in after scene and gripper is empty
    after_empty = SceneObservation(
        gripper_xyz=(0.0, 0.20, 0.06),
        gripper_opening=0.070,
        objects={},
    )
    assert no_target(after_empty, "red_block_1")
    ev_none = evaluate_outcome(before, after_empty, "red_block_1", (0.0, 0.20, 0.06), scene)
    assert ev_none.primary is OutcomeType.NO_Target if hasattr(OutcomeType, "NO_Target") else ev_none.primary is OutcomeType.NO_TARGET

    # 4. table_hit when gripper z <= table_plane_z + 0.001
    after_hit = SceneObservation(
        gripper_xyz=(0.0, 0.20, 0.0002),
        gripper_opening=0.040,
        objects={"red_block_1": (0.0, 0.20, 0.02)},
    )
    assert table_hit(after_hit, table_z=0.0)
    ev_hit = evaluate_outcome(before, after_hit, "red_block_1", (0.0, 0.20, 0.0002), scene)
    assert ev_hit.primary is OutcomeType.TABLE_HIT


def test_object_outside_table_rim_is_knocked_away(scene: dict) -> None:
    """T26 (C18): config/scene.yaml has an active 20 mm rim; an object outside the rim is object_knocked_away."""
    rim = scene["table"]["rim"]
    assert rim["enabled"] is True
    assert float(rim["height"]) == 0.02

    before = SceneObservation(
        gripper_xyz=(0.0, 0.20, 0.02),
        gripper_opening=0.040,
        objects={"red_block_1": (0.0, 0.20, 0.02)},
    )
    # Table is 0.55 x 0.50 centred at (0.0, 0.12), so X spans [-0.275, +0.275] and Y spans [-0.13, +0.37].
    # An object knocked out to (0.31, 0.20, 0.02) is outside the rim:
    after_knocked = SceneObservation(
        gripper_xyz=(0.0, 0.20, 0.04),
        gripper_opening=0.040,
        objects={"red_block_1": (0.31, 0.20, 0.02)},
    )
    assert is_inside_table_rim((0.0, 0.20), scene) is True
    assert is_inside_table_rim((0.31, 0.20), scene) is False
    assert object_knocked_away(before, after_knocked, "red_block_1", scene) is True
    assert object_lifted(before, after_knocked, "red_block_1") is False

    ev = evaluate_outcome(before, after_knocked, "red_block_1", (0.0, 0.20, 0.04), scene)
    assert ev.primary is OutcomeType.OBJECT_KNOCKED_AWAY
