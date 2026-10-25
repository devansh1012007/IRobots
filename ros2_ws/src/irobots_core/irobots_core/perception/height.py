"""Monocular height estimation: Tier P (probe), Tier C (prior), Tier V (vanishing point).

Without a depth sensor, object height is either MEASURED (by descending the
gripper until the jaw occludes the object's top edge) or ASSUMED from a
per-class table, never guessed silently (PLAN:irobots-v2:C15, D.3, D6).

Tier V (vanishing-point / silhouette inversion) is implemented and disabled by
default because the silhouette equation ``h_px = f(H cos theta + D sin theta)/Z``
confounds object height ``H`` with footprint depth ``D`` (+72% bias on a 4 cm
cube at 35 deg). It remains in the codebase so RQ2 can measure and report why
it was rejected.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from typing import Any

from ..config import ConfigError
from .homography import CameraModel


class HeightSource(StrEnum):
    """Which tier produced the height estimate."""

    PROBE = "probe"
    PRIOR = "prior"
    VANISHING_POINT = "vanishing_point"


@dataclass(frozen=True, slots=True)
class HeightEstimate:
    """Estimated object height in metres, with provenance and confidence."""

    value: float
    source: HeightSource
    confidence: float
    provenance: str = "design"
    bias_note: str = ""


def estimate_height_probe(
    probe_samples: Sequence[tuple[float, float]],
    object_top_row_px: float,
    occlusion_row_tolerance_px: float = 3.0,
) -> HeightEstimate:
    """Tier P: measure object height from descending gripper jaw occlusion rows.

    :param probe_samples: ordered sequence of ``(jaw_z_m, jaw_leading_edge_row_px)``
        captured while descending toward the object.
    :param object_top_row_px: image row ``v`` of the object's top-front edge.
    :param occlusion_row_tolerance_px: pixel tolerance for declaring occlusion.
    :returns: a :class:`HeightEstimate` with ``source=HeightSource.PROBE``.
    """
    if not probe_samples:
        raise ValueError("probe_samples must not be empty")

    # Find the sample whose jaw leading edge row is closest to (or first crosses)
    # object_top_row_px within occlusion_row_tolerance_px.
    best_z: float | None = None
    best_err = float("inf")
    for jaw_z, jaw_row in probe_samples:
        err = abs(float(jaw_row) - float(object_top_row_px))
        if err <= occlusion_row_tolerance_px and err < best_err:
            best_err = err
            best_z = float(jaw_z)
    if best_z is None:
        # Interpolate if samples bracket object_top_row_px
        for (z0, r0), (z1, r1) in zip(probe_samples, probe_samples[1:], strict=False):
            if (r0 - object_top_row_px) * (r1 - object_top_row_px) <= 0.0 and abs(r1 - r0) > 1e-6:
                alpha = (object_top_row_px - r0) / (r1 - r0)
                best_z = float(z0 + alpha * (z1 - z0))
                best_err = 0.0
                break

    if best_z is None:
        raise ValueError(
            f"probe did not cross object_top_row_px={object_top_row_px:.1f} within tolerance"
        )

    conf = max(0.8, 1.0 - (best_err / max(occlusion_row_tolerance_px * 5.0, 1.0)))
    return HeightEstimate(
        value=best_z,
        source=HeightSource.PROBE,
        confidence=conf,
        provenance="measured",
    )


def estimate_height_prior(
    label: str,
    priors_table: Mapping[str, Mapping[str, Any]],
) -> HeightEstimate:
    """Tier C: look up the object's class height prior in ``config/perception.yaml``."""
    key = label.strip().lower()
    entry = priors_table.get(label) or priors_table.get(key)
    if entry is None:
        # Fallback to generic "block" or "cube" if present
        for fallback in ("block", "cube"):
            if fallback in priors_table and fallback in key:
                entry = priors_table[fallback]
                break
    if entry is None:
        raise ConfigError(f"no height prior configured for class {label!r}")
    height = float(entry["height"])
    prov = str(entry.get("provenance", "design"))
    return HeightEstimate(
        value=height,
        source=HeightSource.PRIOR,
        confidence=0.70,
        provenance=prov,
    )


def estimate_height_vanishing_point(
    camera: CameraModel,
    bbox_height_px: float,
    object_xy: tuple[float, float],
    tilt_deg: float = 35.0,
) -> HeightEstimate:
    """Tier V: naive single-view silhouette inversion (confounded by footprint depth D).

    Inverts ``h_px = f * (H * cos(theta)) / Z`` ignoring the ``D * sin(theta)``
    footprint contribution, which on a 4 cm cube at 35 deg yields ~6.9 cm (+72%).
    Kept for the RQ2 ablation so the bias is measured rather than hand-waved.
    """
    cam_pos = camera.position
    z_dist = math.sqrt(
        (object_xy[0] - cam_pos[0]) ** 2
        + (object_xy[1] - cam_pos[1]) ** 2
        + (0.0 - cam_pos[2]) ** 2
    )
    cos_t = math.cos(math.radians(tilt_deg))
    if cos_t <= 1e-6:
        raise ValueError("camera tilt approaches 90 deg; vertical projection is degenerate")
    h_est = (float(bbox_height_px) * z_dist) / (camera.focal_px * cos_t)
    return HeightEstimate(
        value=h_est,
        source=HeightSource.VANISHING_POINT,
        confidence=0.25,
        provenance="assumed",
        bias_note=(
            "confounded by footprint depth D*sin(theta); over-estimates a 4cm cube at 35 deg by ~+72%"
        ),
    )


class HeightEstimator:
    """Configurable 3-tier height estimator following ``config/perception.yaml``."""

    def __init__(
        self,
        priors: Mapping[str, Mapping[str, Any]],
        *,
        camera: CameraModel | None = None,
        strategy: str = "probe",
        allow_fallback_to_prior: bool = True,
        enable_vanishing_point: bool = False,
        occlusion_row_tolerance_px: float = 3.0,
    ) -> None:
        self.priors = priors
        self.camera = camera
        self.strategy = strategy
        self.allow_fallback_to_prior = allow_fallback_to_prior
        self.enable_vanishing_point = enable_vanishing_point
        self.occlusion_row_tolerance_px = occlusion_row_tolerance_px

    @classmethod
    def from_config(
        cls,
        perception_cfg: Mapping[str, Any],
        *,
        camera: CameraModel | None = None,
    ) -> HeightEstimator:
        priors = perception_cfg.get("height_priors", {})
        h_cfg = perception_cfg.get("height", {})
        probe_cfg = h_cfg.get("probe", {})
        return cls(
            priors=priors,
            camera=camera,
            strategy=str(h_cfg.get("strategy", "probe")),
            allow_fallback_to_prior=bool(h_cfg.get("allow_fallback_to_prior", True)),
            enable_vanishing_point=bool(h_cfg.get("enable_vanishing_point", False)),
            occlusion_row_tolerance_px=float(probe_cfg.get("occlusion_row_tolerance_px", 3.0)),
        )

    def estimate(
        self,
        label: str,
        *,
        probe_samples: Sequence[tuple[float, float]] | None = None,
        object_top_row_px: float | None = None,
        bbox_height_px: float | None = None,
        object_xy: tuple[float, float] | None = None,
    ) -> HeightEstimate:
        if self.strategy == "probe":
            if probe_samples is not None and object_top_row_px is not None:
                try:
                    return estimate_height_probe(
                        probe_samples,
                        object_top_row_px,
                        self.occlusion_row_tolerance_px,
                    )
                except ValueError:
                    if not self.allow_fallback_to_prior:
                        raise
            if self.allow_fallback_to_prior:
                return estimate_height_prior(label, self.priors)
            raise ValueError("probe height unavailable and fallback to prior is disabled")

        if self.strategy == "prior":
            return estimate_height_prior(label, self.priors)

        if self.strategy == "vanishing_point":
            if bbox_height_px is None or object_xy is None:
                raise ValueError("vanishing_point strategy requires bbox_height_px and object_xy")
            return self.estimate_vanishing_point(bbox_height_px, object_xy)

        raise ConfigError(f"unknown height strategy {self.strategy!r}")

    def estimate_vanishing_point(
        self,
        bbox_height_px: float,
        object_xy: tuple[float, float],
        *,
        force: bool = False,
    ) -> HeightEstimate:
        if not self.enable_vanishing_point and not force:
            raise ValueError(
                "vanishing_point height estimation is disabled by default (see PLAN.md §D.3)"
            )
        if self.camera is None:
            raise ValueError("HeightEstimator requires a CameraModel for vanishing_point")
        return estimate_height_vanishing_point(self.camera, bbox_height_px, object_xy)
