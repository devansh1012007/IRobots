"""Tests for persistent object identity tracking (PLAN:irobots-v2:C14, T18, T19, D.4, I1).

Asserts that tracked object IDs survive detection index reordering across
frames (T18) and that when a tracked ID is lost, skills targeting that ID
are refused with ``unknown_target`` rather than silently re-targeted (T19).
"""

from __future__ import annotations

import pytest

from irobots_core.perception.detector import Detection
from irobots_core.perception.tracker import ObjectTracker, TrackStatus, TrackerConfig
from irobots_core.skills import SkillCall, SkillValidationError, validate_skill_call


def test_tracked_id_survives_three_frame_reordering() -> None:
    """T18: persistent tracked IDs remain bound to the same physical objects even when detection order flips."""
    tracker = ObjectTracker(
        TrackerConfig(iou_threshold=0.25, max_centroid_shift_m=0.03, max_age_frames=3, confirm_frames=2)
    )

    det_red = Detection(label="red block", confidence=0.95, bbox_px=(100.0, 200.0, 140.0, 240.0), index=0)
    det_blue = Detection(label="blue block", confidence=0.90, bbox_px=(300.0, 210.0, 340.0, 250.0), index=1)
    xy_red = (-0.04, 0.20)
    xy_blue = (0.05, 0.21)

    # Frame 1: initial observation (tentative until confirm_frames=2)
    tracks1 = tracker.update([det_red, det_blue], [xy_red, xy_blue])
    id_red = next(t.track_id for t in tracks1 if t.label == "red block")
    id_blue = next(t.track_id for t in tracks1 if t.label == "blue block")
    assert id_red != id_blue

    # Frame 2: detection list order is REVERSED (blue is index 0, red is index 1)
    det_blue_f2 = Detection(label="blue block", confidence=0.98, bbox_px=(301.0, 210.0, 341.0, 250.0), index=0)
    det_red_f2 = Detection(label="red block", confidence=0.88, bbox_px=(101.0, 201.0, 141.0, 241.0), index=1)
    tracks2 = tracker.update([det_blue_f2, det_red_f2], [(0.051, 0.21), (-0.039, 0.20)])
    assert next(t.track_id for t in tracks2 if t.label == "red block") == id_red
    assert next(t.track_id for t in tracks2 if t.label == "blue block") == id_blue
    assert all(t.status is TrackStatus.CONFIRMED for t in tracks2)

    # Frame 3: reordered again with a slight 2mm jitter
    det_red_f3 = Detection(label="red block", confidence=0.92, bbox_px=(100.5, 200.5, 140.5, 240.5), index=0)
    det_blue_f3 = Detection(label="blue block", confidence=0.91, bbox_px=(300.5, 210.5, 340.5, 250.5), index=1)
    tracks3 = tracker.update([det_red_f3, det_blue_f3], [(-0.04, 0.201), (0.05, 0.209)])
    assert next(t.track_id for t in tracks3 if t.label == "red block") == id_red
    assert next(t.track_id for t in tracks3 if t.label == "blue block") == id_blue


def test_lost_tracked_id_causes_skills_to_refuse() -> None:
    """T19: when an object disappears beyond max_age_frames, its ID enters LOST and skills refuse it."""
    tracker = ObjectTracker(
        TrackerConfig(iou_threshold=0.25, max_centroid_shift_m=0.03, max_age_frames=2, confirm_frames=2)
    )
    det_red = Detection(label="red block", confidence=0.95, bbox_px=(100.0, 200.0, 140.0, 240.0), index=0)
    tracker.update([det_red], [(-0.04, 0.20)])
    tracks = tracker.update([det_red], [(-0.04, 0.20)])
    red_id = tracks[0].track_id
    assert red_id in tracker.active_ids()

    # Object disappears for 3 frames (> max_age_frames=2) -> status becomes LOST
    for _ in range(3):
        tracker.update([], [])

    assert red_id not in tracker.active_ids()
    track = tracker.get(red_id)
    assert track is not None
    assert track.status is TrackStatus.LOST

    # Attempting to execute a skill on the lost ID must fail with unknown_target
    with pytest.raises(SkillValidationError, match="unknown_target"):
        validate_skill_call(SkillCall(skill="grasp", target_id=red_id), tracker.active_ids())
