"""Configurable camera frame source: simulation camera vs ESP32-CAM (PLAN:irobots-v2:C65, C67-F3).

Enforces explicit and safe semantics for the two camera sources:
1. ``sim`` (:class:`SimCameraSource`): renders directly from the paused simulator
   (`MuJoCoSim` or `DeterministicKinematicSim`), stamped with simulated time
   ``sim.total_steps * sim.dt`` (Invariant I6), and uses :class:`AnalyticCalibration`
   from ``config/scene.yaml``.
2. ``esp32_cam`` (:class:`ESP32CameraSource`):
   - Requires a validated :class:`MarkerCalibration` loaded from
     ``perception.camera_source.calibration_file`` (at least 4 correspondences,
     reprojection RMS <= ``calibration.max_reprojection_rms_m``). Refuses with
     :class:`ConfigError` if missing or invalid; never falls back to analytic
     calibration.
   - Fetches JPEG frames from ``GET /capture`` configured via ``BOARD_IP``
     (never exposing credentials in ``capture_host``).
   - Returns a typed :class:`CameraCaptureError` on capture/timeout failure while
     honouring per-skill/per-episode deadlines (Invariant I4) so the closed loop
     records a first-class ``camera_unavailable`` outcome rather than crashing or
     hanging.
"""

from __future__ import annotations

import time
import urllib.parse
import urllib.request
from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from typing import Any, Final, Protocol

import cv2
import numpy as np

from ..config import PROVENANCE_VALUES, REPO_ROOT, ConfigError, env_or, load_yaml
from .homography import DEFAULT_MAX_RMS_M, MarkerCalibration

HYBRID_MODE_LABEL: Final[str] = "hybrid: real camera + simulated arm"


class CameraSourceMode(StrEnum):
    """Supported camera frame sources."""

    SIM = "sim"
    ESP32_CAM = "esp32_cam"


@dataclass(frozen=True, slots=True)
class CapturedFrame:
    """One RGB frame captured from either the simulation camera or the ESP32-CAM."""

    rgb: np.ndarray
    timestamp_s: float
    frame_id: int
    source: CameraSourceMode
    host: str = ""
    ok: bool = True


@dataclass(frozen=True, slots=True)
class CameraCaptureError:
    """Typed error returned when a camera capture fails or exceeds its deadline (C67-F3, I4)."""

    reason: str = "camera_unavailable"
    detail: str = ""
    source: CameraSourceMode = CameraSourceMode.ESP32_CAM
    host: str = ""
    timestamp_s: float = 0.0
    ok: bool = False


CaptureOutcome = CapturedFrame | CameraCaptureError


def load_marker_calibration_file(
    calibration_file: str | Path,
    *,
    max_rms_m: float = DEFAULT_MAX_RMS_M,
    min_points: int = 4,
) -> MarkerCalibration:
    """Load and validate a :class:`MarkerCalibration` from a YAML/JSON file.

    Raises :class:`ConfigError` if the path is empty, missing, malformed, has
    fewer than ``min_points`` correspondences, or exceeds ``max_rms_m``.
    """
    raw_str = str(calibration_file).strip()
    if not raw_str:
        raise ConfigError(
            "camera_source 'esp32_cam' requires perception.camera_source.calibration_file "
            "(4+ marker correspondences); refusing to fall back to AnalyticCalibration"
        )
    candidate = Path(raw_str)
    if not candidate.is_absolute():
        repo_rel = REPO_ROOT / candidate
        if repo_rel.exists():
            candidate = repo_rel
    if not candidate.exists():
        raise ConfigError(f"calibration_file does not exist: {candidate}")

    data = load_yaml(candidate)
    img_raw = data.get("image_points")
    tbl_raw = data.get("table_points")
    if not isinstance(img_raw, list) or not isinstance(tbl_raw, list):
        raise ConfigError(
            f"calibration_file {candidate} must define 'image_points' and 'table_points' lists"
        )
    try:
        image_points = [(float(pt[0]), float(pt[1])) for pt in img_raw]
        table_points = [(float(pt[0]), float(pt[1])) for pt in tbl_raw]
        return MarkerCalibration.fit(
            image_points,
            table_points,
            max_rms_m=max_rms_m,
            min_points=min_points,
        )
    except (ValueError, TypeError, IndexError) as exc:
        raise ConfigError(f"invalid MarkerCalibration in {candidate}: {exc}") from exc


@dataclass(frozen=True, slots=True)
class CameraSourceConfig:
    """Typed configuration for switching between ``sim`` and ``esp32_cam``."""

    source: CameraSourceMode
    endpoint_env: str
    capture_path: str
    timeout_s: float
    retries: int
    resize_to_scene: bool
    width: int
    height: int
    calibration_file: str = ""
    allow_real_camera_with_sim_arm: bool = False
    max_reprojection_rms_m: float = DEFAULT_MAX_RMS_M
    min_markers: int = 4
    marker_calibration: MarkerCalibration | None = None
    provenance: str = "design"

    def __post_init__(self) -> None:
        if self.provenance not in PROVENANCE_VALUES:
            raise ConfigError(f"invalid provenance {self.provenance!r} on CameraSourceConfig")
        if self.width <= 0 or self.height <= 0:
            raise ConfigError(f"invalid camera dimensions {self.width}x{self.height}")
        if self.timeout_s <= 0.0:
            raise ConfigError(f"camera timeout_s must be positive, got {self.timeout_s}")
        if self.retries < 1:
            raise ConfigError(f"camera retries must be >= 1, got {self.retries}")
        if self.source is CameraSourceMode.ESP32_CAM and self.marker_calibration is None:
            raise ConfigError(
                "camera_source 'esp32_cam' requires a validated MarkerCalibration; "
                "never falls back to AnalyticCalibration"
            )

    @classmethod
    def from_configs(
        cls,
        perception_cfg: Mapping[str, Any],
        scene_cfg: Mapping[str, Any],
        *,
        source_override: str | CameraSourceMode | None = None,
        calibration_file_override: str | Path | None = None,
        allow_real_camera_with_sim_arm_override: bool | None = None,
        marker_calibration_override: MarkerCalibration | None = None,
    ) -> CameraSourceConfig:
        cs_node = perception_cfg.get("camera_source") or {}
        if not isinstance(cs_node, Mapping):
            raise ConfigError("perception.camera_source must be a mapping")

        raw_mode = (
            str(source_override)
            if source_override is not None
            else (env_or("IROBOTS_CAMERA_SOURCE") or str(cs_node.get("source", "sim")))
        ).strip().lower()
        try:
            mode = CameraSourceMode(raw_mode)
        except ValueError as exc:
            raise ConfigError(
                f"invalid camera source {raw_mode!r}; expected one of {[m.value for m in CameraSourceMode]}"
            ) from exc

        cam_node = scene_cfg.get("camera") or {}
        width = int(cam_node.get("width", 640))
        height = int(cam_node.get("height", 480))
        capture_path = str(cs_node.get("capture_path", "/capture"))
        if not capture_path.startswith("/"):
            capture_path = "/" + capture_path

        cal_node = perception_cfg.get("calibration") or {}
        max_rms_m = float(cal_node.get("max_reprojection_rms_m", DEFAULT_MAX_RMS_M))
        min_markers = int(cal_node.get("min_markers", 4))

        cal_file_str = (
            str(calibration_file_override)
            if calibration_file_override is not None
            else (env_or("IROBOTS_CALIBRATION_FILE") or str(cs_node.get("calibration_file", "")))
        ).strip()

        allow_hybrid = (
            bool(allow_real_camera_with_sim_arm_override)
            if allow_real_camera_with_sim_arm_override is not None
            else bool(cs_node.get("allow_real_camera_with_sim_arm", False))
        )

        marker_cal: MarkerCalibration | None = None
        if mode is CameraSourceMode.ESP32_CAM:
            if marker_calibration_override is not None:
                rms = marker_calibration_override.reprojection_rms_m
                if rms is None or rms > max_rms_m:
                    raise ConfigError(
                        f"MarkerCalibration RMS {rms} exceeds max_reprojection_rms_m {max_rms_m}"
                    )
                marker_cal = marker_calibration_override
            else:
                marker_cal = load_marker_calibration_file(
                    cal_file_str,
                    max_rms_m=max_rms_m,
                    min_points=min_markers,
                )

        return cls(
            source=mode,
            endpoint_env=str(cs_node.get("endpoint_env", "BOARD_IP")),
            capture_path=capture_path,
            timeout_s=float(cs_node.get("timeout_s", 5.0)),
            retries=int(cs_node.get("retries", 3)),
            resize_to_scene=bool(cs_node.get("resize_to_scene", True)),
            width=width,
            height=height,
            calibration_file=cal_file_str,
            allow_real_camera_with_sim_arm=allow_hybrid,
            max_reprojection_rms_m=max_rms_m,
            min_markers=min_markers,
            marker_calibration=marker_cal,
            provenance=str(cs_node.get("provenance", "design")),
        )


class CameraFrameSource(Protocol):
    """Common interface implemented by both ``SimCameraSource`` and ``ESP32CameraSource``."""

    @property
    def mode(self) -> CameraSourceMode: ...

    @property
    def capture_host(self) -> str: ...

    def capture(
        self, *, sim_time_s: float | None = None, deadline_s: float | None = None
    ) -> CaptureOutcome: ...


class SimCameraSource:
    """Captures RGB frames directly from the simulator's camera while paused (Invariant I6)."""

    def __init__(self, sim: Any) -> None:
        if sim is None:
            raise ConfigError("SimCameraSource requires an active simulator instance")
        self.sim = sim
        self._frame_seq: int = 0

    @property
    def mode(self) -> CameraSourceMode:
        return CameraSourceMode.SIM

    @property
    def capture_host(self) -> str:
        return ""

    def capture(
        self, *, sim_time_s: float | None = None, deadline_s: float | None = None
    ) -> CapturedFrame:
        _ = deadline_s
        self._frame_seq += 1
        stamp = (
            float(sim_time_s)
            if sim_time_s is not None
            else float(self.sim.total_steps * self.sim.dt)
        )
        rgb = np.asarray(self.sim.render(), dtype=np.uint8).copy()
        return CapturedFrame(
            rgb=rgb,
            timestamp_s=stamp,
            frame_id=self._frame_seq,
            source=CameraSourceMode.SIM,
            host="",
            ok=True,
        )


def _sanitize_url_and_host(raw_url: str) -> tuple[str, str]:
    parsed = urllib.parse.urlsplit(raw_url)
    scheme = parsed.scheme or "http"
    host = parsed.hostname or ""
    host_port = f"{host}:{parsed.port}" if parsed.port is not None else host
    clean_base = f"{scheme}://{host_port}{parsed.path}".rstrip("/")
    return clean_base, host_port


class ESP32CameraSource:
    """Captures RGB frames from the ESP32-CAM HTTP `/capture` endpoint."""

    def __init__(
        self,
        endpoint: str,
        *,
        capture_path: str = "/capture",
        width: int = 640,
        height: int = 480,
        timeout_s: float = 5.0,
        retries: int = 3,
        resize_to_scene: bool = True,
    ) -> None:
        clean_ep = endpoint.strip()
        if not clean_ep:
            raise ConfigError("ESP32CameraSource requires a non-empty endpoint")
        raw_url = clean_ep if clean_ep.startswith(("http://", "https://")) else f"http://{clean_ep}"
        self.base_url, self._capture_host = _sanitize_url_and_host(raw_url)
        self.capture_path = capture_path if capture_path.startswith("/") else f"/{capture_path}"
        self.width = width
        self.height = height
        self.timeout_s = timeout_s
        self.retries = max(1, int(retries))
        self.resize_to_scene = resize_to_scene
        self._frame_seq: int = 0

    @property
    def mode(self) -> CameraSourceMode:
        return CameraSourceMode.ESP32_CAM

    @property
    def capture_host(self) -> str:
        """Host[:port] of the capture endpoint with any user:password credentials stripped."""
        return self._capture_host

    @property
    def capture_url(self) -> str:
        return f"{self.base_url}{self.capture_path}"

    def capture(
        self, *, sim_time_s: float | None = None, deadline_s: float | None = None
    ) -> CaptureOutcome:
        stamp = float(sim_time_s) if sim_time_s is not None else time.time()
        t_start = time.monotonic()
        last_err: str = "no attempts executed"
        raw_bytes: bytes | None = None

        for attempt in range(self.retries):
            elapsed = time.monotonic() - t_start
            if deadline_s is not None and elapsed >= deadline_s:
                last_err = f"camera deadline ({deadline_s:.3f}s) exceeded after {elapsed:.3f}s"
                break
            req_timeout = (
                min(self.timeout_s, max(0.01, deadline_s - elapsed))
                if deadline_s is not None
                else self.timeout_s
            )
            try:
                with urllib.request.urlopen(self.capture_url, timeout=req_timeout) as resp:  # noqa: S310
                    if resp.status != 200:
                        raise OSError(f"HTTP status {resp.status}")
                    raw_bytes = resp.read()
                break
            except Exception as exc:  # noqa: BLE001
                last_err = str(exc)
                if attempt + 1 < self.retries:
                    rem = (deadline_s - (time.monotonic() - t_start)) if deadline_s is not None else 0.05
                    if rem <= 0.0:
                        break
                    time.sleep(min(0.05, rem))

        if raw_bytes is None:
            return CameraCaptureError(
                reason="camera_unavailable",
                detail=f"failed to capture frame from {self._capture_host} after {self.retries} attempt(s): {last_err}",
                source=CameraSourceMode.ESP32_CAM,
                host=self._capture_host,
                timestamp_s=stamp,
            )

        arr = np.frombuffer(raw_bytes, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return CameraCaptureError(
                reason="camera_unavailable",
                detail=f"failed to decode JPEG bytes from {self._capture_host}",
                source=CameraSourceMode.ESP32_CAM,
                host=self._capture_host,
                timestamp_s=stamp,
            )
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if self.resize_to_scene and (rgb.shape[1] != self.width or rgb.shape[0] != self.height):
            rgb = cv2.resize(rgb, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

        self._frame_seq += 1
        return CapturedFrame(
            rgb=np.asarray(rgb, dtype=np.uint8),
            timestamp_s=stamp,
            frame_id=self._frame_seq,
            source=CameraSourceMode.ESP32_CAM,
            host=self._capture_host,
            ok=True,
        )


def build_camera_source(
    config: CameraSourceConfig,
    *,
    sim: Any | None = None,
    endpoint_override: str | None = None,
) -> CameraFrameSource:
    """Instantiate the configured :class:`CameraFrameSource` (`sim` or `esp32_cam`)."""
    if config.source is CameraSourceMode.SIM:
        return SimCameraSource(sim)

    endpoint = endpoint_override or env_or(config.endpoint_env)
    if not endpoint:
        raise ConfigError(
            f"camera_source is 'esp32_cam' but environment variable {config.endpoint_env!r} is not set"
        )
    return ESP32CameraSource(
        endpoint=endpoint,
        capture_path=config.capture_path,
        width=config.width,
        height=config.height,
        timeout_s=config.timeout_s,
        retries=config.retries,
        resize_to_scene=config.resize_to_scene,
    )
