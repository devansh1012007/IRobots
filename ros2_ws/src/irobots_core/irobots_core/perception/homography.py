"""Pixel to arm-base centimetres.

The legacy pipeline computed ``cm = px / 30.0`` with the origin at the image
corner. That is wrong in three ways at once: the origin is not the arm base,
the axes are not the arm's axes, and a single scalar cannot describe a
perspective projection.

Two calibrations implement the same interface:

* :class:`AnalyticCalibration` - exact, from a known camera model. Used in
  simulation, where the camera pose and the table plane are known.
* :class:`MarkerCalibration` - fitted from four or more correspondences
  between image points and known table positions. Used on real hardware, and
  used in simulation to *measure* how much the marker path costs against the
  analytic one.

Both map an image pixel to a point on the table plane, in the arm base frame.

The subtlety that the legacy pipeline missed: the bottom edge of a box
resting on a table is its FRONT bottom edge, so back-projecting it lands
D/2 toward the camera, not on the footprint centre. :func:`contact_correction`
computes that offset, and the correction is applied, not assumed away.

PLAN:irobots-v2:C13
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Final, Protocol, Sequence

import numpy as np

#: A calibration whose reprojection RMS exceeds this is refused, not used.
#: A calibration that is wrong in a way nobody can see is worse than none.
DEFAULT_MAX_RMS_M: Final[float] = 0.005


class Calibration(Protocol):
    """Maps an image pixel to a point on the table plane, in the arm base frame."""

    def to_arm_base_xy(self, u: float, v: float) -> tuple[float, float]:
        """Map a pixel to (x, y) in metres, in the arm base frame."""
        ...

    @property
    def reprojection_rms_m(self) -> float | None:
        """The RMS of the fit, in metres, or None when the fit is exact."""
        ...


@dataclass(frozen=True, slots=True)
class CameraModel:
    """A pinhole camera: where it is, where it looks, and what it can see.

    ``fov_deg`` is the HORIZONTAL field of view; the vertical one follows
    from the aspect ratio.
    """

    position: tuple[float, float, float]
    target: tuple[float, float, float]
    fov_deg: float
    width: int
    height: int

    @property
    def focal_px(self) -> float:
        """Focal length in pixels. Square pixels, so fx == fy."""
        return (self.width / 2.0) / math.tan(math.radians(self.fov_deg) / 2.0)

    @property
    def principal_point(self) -> tuple[float, float]:
        return (self.width / 2.0, self.height / 2.0)

    @property
    def rotation(self) -> np.ndarray:
        """The camera-to-world rotation matrix.

        The camera looks along its own -Z, with +X right and +Y up, which is
        the same convention MuJoCo uses.
        """
        pos = np.asarray(self.position, dtype=float)
        tgt = np.asarray(self.target, dtype=float)
        forward = tgt - pos
        forward /= np.linalg.norm(forward)

        z_axis = -forward
        up = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(z_axis, up))) > 0.999:
            up = np.array([0.0, 1.0, 0.0])
        x_axis = np.cross(z_axis, up)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        return np.column_stack([x_axis, y_axis, z_axis])

    def ray(self, u: float, v: float) -> np.ndarray:
        """The world-space direction of the ray through pixel (u, v)."""
        cx, cy = self.principal_point
        f = self.focal_px
        local = np.array([(u - cx) / f, (v - cy) / f, -1.0])
        direction = self.rotation @ local
        return direction / np.linalg.norm(direction)

    def project(self, point: Sequence[float]) -> tuple[float, float]:
        """Project a world point into the image. The inverse of ``ray``."""
        world = np.asarray(point, dtype=float)
        cam = self.rotation.T @ (world - np.asarray(self.position, dtype=float))
        if cam[2] >= -1e-9:
            raise ValueError("point is behind the camera")
        cx, cy = self.principal_point
        f = self.focal_px
        return (float(cx + f * cam[0] / -cam[2]), float(cy + f * cam[1] / -cam[2]))


@dataclass(frozen=True, slots=True)
class AnalyticCalibration:
    """Exact pixel-to-plane mapping from a known camera model.

    Used in simulation, where the camera pose and the table plane are known.
    The error is not "small", it is zero up to floating point.
    """

    camera: CameraModel
    table_z: float = 0.0

    def to_arm_base_xy(self, u: float, v: float) -> tuple[float, float]:
        origin = np.asarray(self.camera.position, dtype=float)
        direction = self.camera.ray(u, v)
        if abs(direction[2]) < 1e-12:
            raise ValueError("ray is parallel to the table plane")
        t = (self.table_z - origin[2]) / direction[2]
        if t <= 0.0:
            raise ValueError("ray does not hit the table plane in front of the camera")
        point = origin + t * direction
        return (float(point[0]), float(point[1]))

    @property
    def reprojection_rms_m(self) -> float | None:
        return None


@dataclass(frozen=True, slots=True)
class MarkerCalibration:
    """A homography fitted from image points to known table positions.

    This is the path real hardware uses, because on hardware the camera pose
    is not known to the precision the analytic path needs.
    """

    homography: np.ndarray
    reprojection_rms_m: float | None = None

    def to_arm_base_xy(self, u: float, v: float) -> tuple[float, float]:
        p = self.homography @ np.array([u, v, 1.0])
        if abs(p[2]) < 1e-12:
            raise ValueError("degenerate homography")
        return (float(p[0] / p[2]), float(p[1] / p[2]))

    def project(self, u: float, v: float) -> tuple[float, float]:
        """The inverse: where a table position lands in the image."""
        inv = np.linalg.inv(self.homography)
        p = inv @ np.array([u, v, 1.0])
        if abs(p[2]) < 1e-12:
            raise ValueError("degenerate homography")
        return (float(p[0] / p[2]), float(p[1] / p[2]))

    @classmethod
    def fit(
        cls,
        image_points: Sequence[tuple[float, float]],
        table_points: Sequence[tuple[float, float]],
        *,
        max_rms_m: float = DEFAULT_MAX_RMS_M,
        min_points: int = 4,
    ) -> "MarkerCalibration":
        """Fit the homography, and refuse a fit that is not good enough.

        :param image_points: pixel coordinates of the markers.
        :param table_points: the same markers' positions in the arm base
            frame, in metres.
        :param max_rms_m: refuse the calibration above this reprojection RMS.
        :param min_points: the minimum number of correspondences.
        :raises ValueError: if there are too few points, the fit is
            degenerate, or the RMS is too large.
        """
        if len(image_points) != len(table_points):
            raise ValueError("image_points and table_points must be the same length")
        if len(image_points) < min_points:
            raise ValueError(
                f"a homography needs at least {min_points} correspondences, got {len(image_points)}"
            )

        src = np.asarray(image_points, dtype=float).reshape(-1, 2)
        dst = np.asarray(table_points, dtype=float).reshape(-1, 2)

        # Direct linear transform. The system is homogeneous: for each
        # correspondence, w*[x, y, 1]^T = H*[u, v, 1]^T with w free, so the
        # right-hand side is ZERO and the solution is the null space of the
        # 2N x 9 system. (Putting the coordinates on the right-hand side
        # instead solves an affine map, which cannot represent the
        # perspective distortion of a tilted camera - and the RMS check
        # below catches exactly that mistake.)
        a = np.zeros((2 * len(src), 9), dtype=float)
        for i, (u, v) in enumerate(src):
            x, y = dst[i]
            a[2 * i] = [u, v, 1, 0, 0, 0, -u * x, -v * x, -x]
            a[2 * i + 1] = [0, 0, 0, u, v, 1, -u * y, -v * y, -y]
        _u, _s, vh = np.linalg.svd(a)
        h = vh[-1].reshape(3, 3)
        if abs(h[2, 2]) < 1e-12:
            raise ValueError("degenerate homography")

        # Reprojection error, in metres.
        errors = []
        for (u, v), (x, y) in zip(src, dst, strict=True):
            p = h @ np.array([u, v, 1.0])
            if abs(p[2]) < 1e-12:
                raise ValueError("degenerate homography")
            errors.append(float(np.hypot(p[0] / p[2] - x, p[1] / p[2] - y)))
        rms = float(np.sqrt(np.mean(np.square(errors))))

        if rms > max_rms_m:
            raise ValueError(
                f"calibration RMS {rms * 1000:.2f}mm exceeds the {max_rms_m * 1000:.2f}mm limit; "
                "re-measure the marker positions rather than using a bad calibration"
            )
        return cls(homography=h, reprojection_rms_m=rms)


def contact_correction(
    camera_xy: tuple[float, float],
    object_xy: tuple[float, float],
    footprint_m: float,
) -> tuple[float, float]:
    """The offset between a box's front bottom edge and its footprint centre.

    The bottom edge of an axis-aligned box resting on a table is its FRONT
    bottom edge, so back-projecting that edge lands D/2 toward the camera
    rather than on the footprint centre. For a 4 cm cube that is 2.0 cm: the
    difference between grasping an object and missing it by its own width.

    The half-extent of a square of side ``footprint_m`` measured along an
    arbitrary horizontal direction is ``(side/2) * (|cos t| + |sin t|)``, where
    ``t`` is the angle between that direction and the square's axis.

    :returns: the (dx, dy) to ADD to the back-projected edge to get the
        footprint centre.
    """
    dx = camera_xy[0] - object_xy[0]
    dy = camera_xy[1] - object_xy[1]
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        return (0.0, 0.0)
    ux, uy = dx / norm, dy / norm

    # The camera looks roughly along +y, so a direction along the world x
    # axis is roughly perpendicular to the image's x axis. Work the angle out
    # in the image frame by projecting the direction onto the table.
    theta = math.atan2(uy, ux)
    half_extent = 0.5 * footprint_m * (abs(math.cos(theta)) + abs(math.sin(theta)))
    return (-half_extent * ux, -half_extent * uy)
