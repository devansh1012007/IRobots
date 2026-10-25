"""Tests for object detection: GeometricDetector and WorldDetector adapter (PLAN:irobots-v2:C12, T13, T14).

Asserts that known objects in the workspace are detected with class/colour
labels and bounding boxes (T13), and that empty scenes or unseen classes
produce no hallucinated detections (T14).
"""

from __future__ import annotations

import numpy as np

from irobots_core.perception.detector import Detection, filter_detections_by_vocabulary
from irobots_core.perception.geometric import GeometricDetector, colour_name
from irobots_core.perception.homography import CameraModel


def _make_camera(scene: dict) -> CameraModel:
    cam_cfg = scene["camera"]
    return CameraModel(
        position=tuple(float(c) for c in cam_cfg["position"]),  # type: ignore[arg-type]
        target=tuple(float(c) for c in cam_cfg["target"]),  # type: ignore[arg-type]
        fov_deg=float(cam_cfg["hfov_deg"]),
        width=int(cam_cfg["width"]),
        height=int(cam_cfg["height"]),
    )


def _blank_table_frame(camera: CameraModel) -> np.ndarray:
    return np.full((camera.height, camera.width, 3), (97, 99, 107), dtype=np.uint8)


def test_known_object_is_detected_with_label_and_box(robot, scene: dict) -> None:
    """T13: a known red block inside the workspace is detected with label and accurate bottom edge."""
    camera = _make_camera(scene)
    det = GeometricDetector(
        camera=camera,
        workspace_centre=robot.safety.workspace_centre,
        workspace_radius=robot.safety.workspace_radius,
    )
    frame = _blank_table_frame(camera)
    u, v = camera.project((0.0, 0.20, 0.0))
    u_i, v_i = int(round(u)), int(round(v))
    # Draw a 30x30 red block whose bottom edge sits at v_i
    frame[v_i - 30 : v_i, u_i - 15 : u_i + 15] = (220, 50, 50)

    detections = det.detect(frame)
    assert len(detections) == 1
    d = detections[0]
    assert isinstance(d, Detection)
    assert d.label == "red"
    bu, bv = d.bottom_center_px
    assert abs(bu - u_i) <= 1.5
    assert abs(bv - v_i) <= 1.5


def test_unknown_class_or_empty_scene_returns_no_hallucination(robot, scene: dict) -> None:
    """T14: an empty table or an unseen vocabulary class produces zero detections, never a hallucination."""
    camera = _make_camera(scene)
    det = GeometricDetector(
        camera=camera,
        workspace_centre=robot.safety.workspace_centre,
        workspace_radius=robot.safety.workspace_radius,
    )
    frame = _blank_table_frame(camera)
    # 1. Empty table -> 0 detections
    assert det.detect(frame) == []

    # 2. Red object present, but vocabulary only asks for ["purple sphere"] -> 0 detections
    u, v = camera.project((0.0, 0.20, 0.0))
    u_i, v_i = int(round(u)), int(round(v))
    frame[v_i - 30 : v_i, u_i - 15 : u_i + 15] = (220, 50, 50)
    raw_dets = det.detect(frame)
    assert len(raw_dets) == 1
    filtered = filter_detections_by_vocabulary(raw_dets, ["purple sphere", "green cylinder"])
    assert filtered == []

    # 3. Vocabulary includes "red block" -> matched and labelled "red block"
    matched = filter_detections_by_vocabulary(raw_dets, ["red block", "blue block"])
    assert len(matched) == 1
    assert matched[0].label == "red block"
    assert colour_name((220, 50, 50)) == "red"
