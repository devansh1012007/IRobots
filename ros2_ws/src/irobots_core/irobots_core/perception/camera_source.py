"""Configurable camera frame source: simulation camera vs ESP32-CAM (PLAN:irobots-v2:C65).

Allows the entire perception, planning, worker, and verification pipeline to
operate identically on frames captured from:
1. ``sim`` (:class:`SimCameraSource`): renders directly from the paused simulator
   (`MuJoCoSim` or `DeterministicKinematicSim`), stamped with simulated time
   ``sim.total_steps * sim.dt`` (Invariant I6).
2. ``esp32_cam`` (:class:`ESP32CameraSource`): fetches a single JPEG frame from
   the ESP32-CAM ``GET /capture`` HTTP endpoint configured via ``BOARD_IP`` (no
   hardcoded IP or port in source, per Hard Rule §4 / T12), converts BGR to RGB,
   and resizes to the scene camera's ``640x480`` perception floor if needed.

The active camera source is configured in ``config/perception.yaml`` under
``perception.camera_source.source`` (``sim`` or ``esp32_cam``) and can be
overridden via the ``IROBOTS_CAMERA_SOURCE`` environment variable.
"""

from __future__ import annotations

import time
import urllib.request
from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from typing import Any, Protocol

import cv2
import numpy as np

from ..config import PROVENANCE_VALUES, ConfigError, env_or


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

    @classmethod
    def from_configs(
        cls,
        perception_cfg: Mapping[str, Any],
        scene_cfg: Mapping[str, Any],
        *,
        source_override: str | CameraSourceMode | None = None,
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

        return cls(
            source=mode,
            endpoint_env=str(cs_node.get("endpoint_env", "BOARD_IP")),
            capture_path=capture_path,
            timeout_s=float(cs_node.get("timeout_s", 5.0)),
            retries=int(cs_node.get("retries", 3)),
            resize_to_scene=bool(cs_node.get("resize_to_scene", True)),
            width=width,
            height=height,
            provenance=str(cs_node.get("provenance", "design")),
        )


class CameraFrameSource(Protocol):
    """Common interface implemented by both ``SimCameraSource`` and ``ESP32CameraSource``."""

    @property
    def mode(self) -> CameraSourceMode: ...

    def capture(self, *, sim_time_s: float | None = None) -> CapturedFrame: ...


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

    def capture(self, *, sim_time_s: float | None = None) -> CapturedFrame:
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
        )


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
        if clean_ep.startswith(("http://", "https://")):
            self.base_url = clean_ep.rstrip("/")
        else:
            self.base_url = f"http://{clean_ep.rstrip('/')}"
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
    def capture_url(self) -> str:
        return f"{self.base_url}{self.capture_path}"

    def capture(self, *, sim_time_s: float | None = None) -> CapturedFrame:
        last_err: Exception | None = None
        raw_bytes: bytes | None = None
        for attempt in range(self.retries):
            try:
                with urllib.request.urlopen(self.capture_url, timeout=self.timeout_s) as resp:  # noqa: S310
                    if resp.status != 200:
                        raise OSError(f"HTTP status {resp.status}")
                    raw_bytes = resp.read()
                break
            except Exception as exc:  # noqa: BLE001
                last_err = exc
                if attempt + 1 < self.retries:
                    time.sleep(0.05)

        if raw_bytes is None:
            raise RuntimeError(
                f"failed to capture frame from {self.capture_url} after {self.retries} attempt(s): {last_err}"
            )

        arr = np.frombuffer(raw_bytes, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError(f"failed to decode JPEG bytes from {self.capture_url}")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if self.resize_to_scene and (rgb.shape[1] != self.width or rgb.shape[0] != self.height):
            rgb = cv2.resize(rgb, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

        self._frame_seq += 1
        stamp = float(sim_time_s) if sim_time_s is not None else time.time()
        return CapturedFrame(
            rgb=np.asarray(rgb, dtype=np.uint8),
            timestamp_s=stamp,
            frame_id=self._frame_seq,
            source=CameraSourceMode.ESP32_CAM,
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
