"""Open-vocabulary detection.

The legacy pipeline used SAM, which returns anonymous masks with **no class
label**. That is why the agent could not select an object: "the red block"
could not be bound to anything, and the model had to guess from pixels.

YOLO-World gives a label, a box and a confidence, which is what the planner
and the worker need in order to refer to an object by identity.

PLAN:irobots-v2:C12
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from collections.abc import Sequence
from typing import Final

import numpy as np

#: The default open-vocabulary detector. The `s` variant is the smallest
#: that still holds its accuracy, which matters on a CPU-only machine.
DEFAULT_MODEL: Final[str] = "yolov8s-world.pt"


@dataclass(frozen=True, slots=True)
class Detection:
    """One detected object."""

    label: str
    confidence: float
    #: Bounding box in pixels: (x1, y1, x2, y2).
    bbox_px: tuple[float, float, float, float]
    #: The detection's index within this frame. NOT an identity: an id that
    #: survives across frames is the tracker's job.
    index: int
    mask: np.ndarray | None = None

    @property
    def center_px(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.bbox_px
        return (0.5 * (x1 + x2), 0.5 * (y1 + y2))

    @property
    def bottom_center_px(self) -> tuple[float, float]:
        """The midpoint of the BOTTOM edge.

        For a box resting on a table this is the front bottom edge, which is
        the object's contact point with the table - so it is the right thing
        to back-project onto the table plane. It is offset from the footprint
        centre by D/2 toward the camera, and
        :func:`irobots_core.perception.homography.contact_correction`
        computes that offset.
        """
        x1, _y1, x2, y2 = self.bbox_px
        return (0.5 * (x1 + x2), y2)

    @property
    def width_px(self) -> float:
        x1, _y1, x2, _y2 = self.bbox_px
        return x2 - x1

    @property
    def height_px(self) -> float:
        _x1, y1, _x2, y2 = self.bbox_px
        return y2 - y1


class WorldDetector:
    """A thin wrapper over Ultralytics' YOLO-World.

    The vocabulary is set per task, not baked into the model. Setting it
    re-encodes the text embeddings, which is not free, so the vocabulary is
    cached and only recomputed when it actually changes.
    """

    def __init__(
        self,
        weights: str | Path | None = None,
        *,
        imgsz: int = 640,
        conf_threshold: float = 0.30,
        iou_threshold: float = 0.55,
        device: str = "cpu",
    ) -> None:
        from ultralytics import YOLOWorld

        if weights is None:
            weights = Path(__file__).resolve().parents[4] / "weights" / DEFAULT_MODEL
        self.weights = str(weights)
        self.imgsz = imgsz
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.device = device
        self._model = YOLOWorld(self.weights)
        self._vocabulary: tuple[str, ...] = ()

    @property
    def vocabulary(self) -> tuple[str, ...]:
        return self._vocabulary

    def set_vocabulary(self, vocabulary: Sequence[str]) -> None:
        """Set the open-vocabulary vocabulary. Cheap if it has not changed."""
        new = tuple(vocabulary)
        if new == self._vocabulary:
            return
        self._model.set_classes(list(new))
        self._vocabulary = new

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """Detect objects in an RGB frame.

        :param frame: an HxWx3 uint8 RGB image.
        :returns: the detections, most confident first.
        """
        results = self._model.predict(
            frame,
            imgsz=self.imgsz,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False,
        )
        if not results:
            return []
        result = results[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []

        names = result.names
        out: list[Detection] = []
        for i in range(len(boxes)):
            x1, y1, x2, y2 = (float(v) for v in boxes.xyxy[i].tolist())
            conf = float(boxes.conf[i])
            cls = int(boxes.cls[i])
            label = names[cls] if isinstance(names, dict) else str(names[cls])
            out.append(
                Detection(
                    label=str(label),
                    confidence=conf,
                    bbox_px=(x1, y1, x2, y2),
                    index=len(out),
                )
            )
        out.sort(key=lambda d: -d.confidence)
        # Re-index after the sort so the index is the detection's rank.
        return [
            Detection(d.label, d.confidence, d.bbox_px, i, d.mask) for i, d in enumerate(out)
        ]


def timed_detect(
    detector: WorldDetector, frame: np.ndarray
) -> tuple[list[Detection], float]:
    """Detect and report the elapsed wall-clock time, in seconds."""
    start = time.perf_counter()
    detections = detector.detect(frame)
    return detections, time.perf_counter() - start



def filter_detections_by_vocabulary(
    detections: Sequence[Detection],
    vocabulary: Sequence[str],
) -> list[Detection]:
    """Bind detections against the task's open-vocabulary list (PLAN:irobots-v2:C12, T13, T14).

    Earlier vocabulary entries win ties, matching ``config/perception.yaml``.
    """
    vocab = [v.strip() for v in vocabulary if v.strip()]
    if not vocab:
        return []
    out: list[Detection] = []
    for d in detections:
        lbl = d.label.lower().strip()
        matched_label: str | None = None
        for v in vocab:
            vl = v.lower()
            if lbl == vl or vl.startswith(lbl + " ") or (lbl in ("cube", "block") and vl in ("cube", "block")):
                matched_label = v
                break
        if matched_label is not None:
            out.append(
                Detection(
                    label=matched_label,
                    confidence=d.confidence,
                    bbox_px=d.bbox_px,
                    index=len(out),
                    mask=d.mask,
                )
            )
    return out
