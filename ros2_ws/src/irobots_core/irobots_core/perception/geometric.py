"""A geometric detector: find the objects, without a learned model.

**Why this exists.** YOLO-World scores 0.96 on a real photograph of the
arm and finds *nothing* in a MuJoCo render - not a red cube, not a 15 cm
cube, not a textured one, not one with sensor noise and JPEG artifacts. The
flat, uniformly lit primitives of a simulation are simply outside the
detector's training distribution. A learned open-vocabulary detector is
therefore not a dependable part of the critical path, in simulation or (very
likely) on a real camera pointed at small blocks.

What *is* dependable is geometry. The workspace is a known region of a known
table, the camera is calibrated, and the objects are things that are not
table-coloured. That is a deterministic, domain-agnostic detection problem,
and it works identically on a render and on a photograph.

The learned detector is kept, and measured, as a *baseline* - see
``detector.py`` and the RQ4 comparison. It is not on the critical path.

The geometric detector reports a **colour name** rather than a semantic
class. Naming the object ("red block") is the LLM's job, on the annotated
image, where VLMs are strong. The detector's job is to say *where* each
distinct object is, and that is what it does.

PLAN:irobots-v2:C12
"""

from __future__ import annotations

import colorsys
import math
from dataclasses import dataclass
from typing import Final, Sequence

import numpy as np

from .detector import Detection

#: The named colours, in hue order, with the boundaries between them. The
#: hues are in degrees.
COLOUR_BANDS: Final[tuple[tuple[str, float, float], ...]] = (
    ("red", 345.0, 15.0),
    ("orange", 15.0, 45.0),
    ("yellow", 45.0, 70.0),
    ("green", 70.0, 160.0),
    ("cyan", 160.0, 200.0),
    ("blue", 200.0, 255.0),
    ("purple", 255.0, 290.0),
    ("magenta", 290.0, 345.0),
)

#: The maximum distance between two RGB colours, used to normalise the
#: "is this the table?" test.
MAX_RGB_DISTANCE: Final[float] = 441.67


def colour_name(rgb: Sequence[float]) -> str:
    """Name a colour from its RGB value.

    Low-saturation colours are named by value instead ("white", "grey",
    "black"), because a desaturated pixel has no meaningful hue.
    """
    r, g, b = (float(c) for c in rgb[:3])
    h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    if s < 0.18:
        if v > 0.75:
            return "white"
        if v < 0.22:
            return "black"
        return "grey"
    hue_deg = h * 360.0
    for name, low, high in COLOUR_BANDS:
        if low <= high:
            if low <= hue_deg < high:
                return name
        else:  # the band wraps around 0 degrees
            if hue_deg >= low or hue_deg < high:
                return name
    return "grey"


@dataclass(frozen=True, slots=True)
class GeometricDetectorConfig:
    """Tuning for the geometric detector."""

    min_area_px: int = 40
    max_area_frac: float = 0.35
    #: How far a pixel's colour may be from the table's, as a fraction of the
    #: maximum RGB distance, before it counts as an object.
    colour_distance_threshold: float = 0.10
    #: A shadow is the table's own colour, darkened. It is excluded by
    #: matching the table's hue and saturation while being DARKER than the
    #: table. Without this, a shadow is detected as part of the object and
    #: the object's contact point lands on the shadow's edge, tens of
    #: millimetres away from the object.
    hue_tolerance_deg: float = 20.0
    saturation_tolerance: float = 25.0
    shadow_value_margin: float = 12.0


class GeometricDetector:
    """Finds the distinct objects on the table, by appearance.

    :param camera: the calibrated camera, used to find the workspace's image
        region and to convert the scene's table colour into image space.
    :param workspace_centre: the workspace centre in the arm base frame.
    :param workspace_radius: the workspace radius in the arm base frame.
    :param table_rgb: the table's colour. If None, it is estimated from the
        frame's border region, which is the table in every scene we build.
    """

    def __init__(
        self,
        camera,
        workspace_centre: tuple[float, float],
        workspace_radius: float,
        *,
        table_rgb: Sequence[float] | None = None,
        config: GeometricDetectorConfig | None = None,
    ) -> None:
        import cv2

        self._cv2 = cv2
        self.camera = camera
        self.workspace_centre = workspace_centre
        self.workspace_radius = workspace_radius
        self.config = config or GeometricDetectorConfig()
        self._table_rgb: tuple[float, float, float] | None = (
            (float(table_rgb[0]), float(table_rgb[1]), float(table_rgb[2]))
            if table_rgb is not None
            else None
        )
        self._workspace_mask = self._build_workspace_mask()

    # -- setup ------------------------------------------------------------

    def _build_workspace_mask(self) -> np.ndarray:
        """The image mask of the workspace disk."""
        cv2 = self._cv2
        cx, cy = self.workspace_centre
        radius = self.workspace_radius
        points = []
        for i in range(48):
            angle = 2.0 * math.pi * i / 48.0
            points.append(
                self.camera.project((cx + radius * math.cos(angle), cy + radius * math.sin(angle), 0.0))
            )
        mask = np.zeros((self.camera.height, self.camera.width), dtype=np.uint8)
        poly = np.array([[int(round(u)), int(round(v))] for u, v in points], dtype=np.int32)
        cv2.fillPoly(mask, [poly], 255)
        return mask

    def _estimate_table_colour(self, frame: np.ndarray) -> tuple[float, float, float]:
        """The table's colour, from the MEDIAN of the workspace region.

        Not from the image border: the border of a downward-looking camera is
        the sky, not the table, and taking the border's median estimates the
        sky's colour - which then makes the entire table read as foreground.

        The median of the workspace is the table, because the table is most
        of that region and the objects are a small minority of it. The
        median is also robust to a bright object touching the edge.
        """
        if self._table_rgb is not None:
            return self._table_rgb
        pixels = frame[self._workspace_mask > 0]
        if pixels.size == 0:
            return (128.0, 128.0, 128.0)
        med = np.median(pixels.reshape(-1, 3), axis=0)
        self._table_rgb = (float(med[0]), float(med[1]), float(med[2]))
        return self._table_rgb

    # -- detection --------------------------------------------------------

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """Find the objects in an RGB frame.

        :param frame: an HxWx3 uint8 RGB image.
        :returns: one :class:`Detection` per distinct object, largest first.
        """
        cv2 = self._cv2
        cfg = self.config

        table_rgb = self._estimate_table_colour(frame)

        # Foreground = inside the workspace, not table-coloured, and not a
        # shadow.
        #
        # The "not table-coloured" test is a plain RGB distance, not a hue
        # test: a hue test cannot see a WHITE object on a grey table, because
        # both have no meaningful hue - and white blocks are perfectly
        # ordinary task objects.
        inside = self._workspace_mask > 0
        diff = frame.astype(np.float32) - np.asarray(table_rgb, dtype=np.float32)
        distance = np.sqrt((diff * diff).sum(axis=2))
        differs = distance > cfg.colour_distance_threshold * 441.67

        # Exclude the shadow: the table's own hue and saturation, but darker.
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV).astype(np.float32)
        table_hsv = cv2.cvtColor(
            np.uint8([[[int(round(c)) for c in table_rgb]]]), cv2.COLOR_RGB2HSV
        ).astype(np.float32)[0, 0]
        dh = np.abs(hsv[:, :, 0] - float(table_hsv[0]))
        dh = np.minimum(dh, 180.0 - dh)
        shadow_like = (
            (dh <= cfg.hue_tolerance_deg)
            & (hsv[:, :, 1] <= float(table_hsv[1]) + cfg.saturation_tolerance)
            & (hsv[:, :, 2] < float(table_hsv[2]) - cfg.shadow_value_margin)
        )

        mask = (inside & differs & ~shadow_like).astype(np.uint8) * 255
        # Close small gaps so a shaded face does not split an object in two.
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        total_px = float(self.camera.width * self.camera.height)
        out: list[Detection] = []
        for i in range(1, n_labels):
            x, y, w, h, area = (int(v) for v in stats[i])
            if area < cfg.min_area_px or area > cfg.max_area_frac * total_px:
                continue
            component = labels == i
            mean_rgb = frame[component].reshape(-1, 3).mean(axis=0)
            # The contact point is the BOTTOM of the component, in the middle
            # of its bottom row: the object's contact with the table.
            rows = np.where(component.any(axis=1))[0]
            bottom_row = int(rows[-1])
            bottom_cols = np.where(component[bottom_row])[0]
            cx_px = float(bottom_cols.mean())
            out.append(
                Detection(
                    label=colour_name(mean_rgb),
                    confidence=1.0,
                    bbox_px=(float(x), float(y), float(x + w), float(y + h)),
                    index=len(out),
                )
            )
        out.sort(key=lambda d: -(d.bbox_px[2] - d.bbox_px[0]) * (d.bbox_px[3] - d.bbox_px[1]))
        return [
            Detection(d.label, d.confidence, d.bbox_px, i) for i, d in enumerate(out)
        ]
