"""Persistent object identity tracking across frames (PLAN:irobots-v2:C14, D.4).

A per-frame detection index is broken: "the red block" is index 0 this frame
and index 1 next frame. This tracker maintains persistent IDs using bounding-box
IoU + table-plane centroid distance + appearance/label consistency, and marks
tracks as ``LOST`` when unseen for more than ``max_age_frames`` so skills
refuse unknown/lost IDs rather than silently re-targeting.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from enum import StrEnum

from .detector import Detection


class TrackStatus(StrEnum):
    """Lifecycle state of a tracked object identity."""

    TENTATIVE = "tentative"
    CONFIRMED = "confirmed"
    LOST = "lost"


@dataclass(frozen=True, slots=True)
class TrackerConfig:
    """Parameters loaded from ``config/perception.yaml`` ``perception.tracker``."""

    iou_threshold: float = 0.25
    max_centroid_shift_m: float = 0.03
    max_age_frames: int = 5
    confirm_frames: int = 2


@dataclass(slots=True)
class TrackedObject:
    """One tracked physical object with a persistent ID."""

    track_id: str
    label: str
    bbox_px: tuple[float, float, float, float]
    xy_m: tuple[float, float]
    confidence: float
    hits: int = 1
    age_since_seen: int = 0
    status: TrackStatus = TrackStatus.TENTATIVE


def bbox_iou(
    box_a: tuple[float, float, float, float],
    box_b: tuple[float, float, float, float],
) -> float:
    """Intersection-over-Union of two ``(x1, y1, x2, y2)`` bounding boxes."""
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


class ObjectTracker:
    """Maintains persistent object IDs across frames."""

    def __init__(self, config: TrackerConfig | None = None) -> None:
        self.config = config or TrackerConfig()
        self._tracks: dict[str, TrackedObject] = {}
        self._next_seq: int = 1

    def reset(self) -> None:
        self._tracks.clear()
        self._next_seq = 1

    def get(self, track_id: str) -> TrackedObject | None:
        return self._tracks.get(track_id)

    def active_tracks(self) -> list[TrackedObject]:
        """Return tracks that are currently visible (not LOST)."""
        return [
            t
            for t in self._tracks.values()
            if t.status in (TrackStatus.CONFIRMED, TrackStatus.TENTATIVE) and t.age_since_seen == 0
        ]

    def active_ids(self) -> set[str]:
        """Return the set of valid target IDs that skills may currently reference."""
        return {
            t.track_id
            for t in self._tracks.values()
            if t.status is not TrackStatus.LOST
        }

    def update(
        self,
        detections: Sequence[Detection],
        xy_positions_m: Sequence[tuple[float, float]],
    ) -> list[TrackedObject]:
        """Match new frame detections to existing tracks and return active tracks."""
        if len(detections) != len(xy_positions_m):
            raise ValueError("detections and xy_positions_m must have the same length")

        cfg = self.config
        candidate_tracks = [
            t for t in self._tracks.values() if t.status is not TrackStatus.LOST
        ]
        matched_track_ids: set[str] = set()
        matched_det_indices: set[int] = set()

        # Score all (track, detection) pairs that share a compatible label and
        # satisfy either the IoU threshold or the centroid shift limit.
        pairs: list[tuple[float, str, int]] = []
        for track in candidate_tracks:
            for idx, (det, xy) in enumerate(zip(detections, xy_positions_m, strict=True)):
                if track.label != det.label:
                    continue
                iou = bbox_iou(track.bbox_px, det.bbox_px)
                shift = math.hypot(track.xy_m[0] - xy[0], track.xy_m[1] - xy[1])
                if iou < cfg.iou_threshold and shift > cfg.max_centroid_shift_m:
                    continue
                score = iou - (shift / max(cfg.max_centroid_shift_m, 1e-6)) * 0.25
                pairs.append((score, track.track_id, idx))

        pairs.sort(key=lambda item: -item[0])
        for _score, tid, idx in pairs:
            if tid in matched_track_ids or idx in matched_det_indices:
                continue
            matched_track_ids.add(tid)
            matched_det_indices.add(idx)
            det = detections[idx]
            xy = xy_positions_m[idx]
            tr = self._tracks[tid]
            tr.bbox_px = det.bbox_px
            tr.xy_m = (float(xy[0]), float(xy[1]))
            tr.confidence = det.confidence
            tr.hits += 1
            tr.age_since_seen = 0
            if tr.hits >= cfg.confirm_frames:
                tr.status = TrackStatus.CONFIRMED

        # Unmatched existing tracks age; if they exceed max_age_frames they become LOST
        for track in candidate_tracks:
            if track.track_id not in matched_track_ids:
                track.age_since_seen += 1
                if track.age_since_seen > cfg.max_age_frames:
                    track.status = TrackStatus.LOST

        # Unmatched detections spawn new tracks with unique persistent IDs
        for idx, (det, xy) in enumerate(zip(detections, xy_positions_m, strict=True)):
            if idx in matched_det_indices:
                continue
            slug = det.label.lower().replace(" ", "_")
            tid = f"{slug}_{self._next_seq}"
            self._next_seq += 1
            status = (
                TrackStatus.CONFIRMED if cfg.confirm_frames <= 1 else TrackStatus.TENTATIVE
            )
            self._tracks[tid] = TrackedObject(
                track_id=tid,
                label=det.label,
                bbox_px=det.bbox_px,
                xy_m=(float(xy[0]), float(xy[1])),
                confidence=det.confidence,
                hits=1,
                age_since_seen=0,
                status=status,
            )

        return self.active_tracks()
