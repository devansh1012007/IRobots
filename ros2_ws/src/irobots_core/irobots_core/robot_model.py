"""The robot, as data.

``config/robot.yaml`` is the single source of truth for the arm's geometry,
joint limits and servo mapping. This module turns it into typed objects and
enforces the invariants the rest of the system relies on.

The arm is NOT physically assembled. Every grasp claim in this project is a
claim about the robot described here, not about hardware.

PLAN:irobots-v2:C06
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Final

from .config import ConfigError, Scalar, load_yaml, scalar_from

#: Joint names, in chain order from the base outwards.
#:
#: The three pitch joints are planar. ``wrist_yaw`` sits at the TOOL TIP and
#: rotates about the approach axis, so for a downward-pointing tool it moves
#: the tip not at all and purely sets the jaw closing direction.
JOINT_ORDER: Final[tuple[str, ...]] = (
    "base_yaw",
    "shoulder_pitch",
    "elbow_pitch",
    "wrist_pitch",
    "wrist_yaw",
)

#: The actuated gripper channel, separate from the arm joints.
GRIPPER_CHANNEL: Final[str] = "gripper"


@dataclass(frozen=True, slots=True)
class JointSpec:
    """One revolute joint."""

    name: str
    axis: tuple[float, float, float]
    parent: str
    child: str
    origin: tuple[float, float, float]
    min_deg: float
    max_deg: float
    max_velocity: float
    max_acceleration: float
    provenance: str

    @property
    def min_rad(self) -> float:
        return math.radians(self.min_deg)

    @property
    def max_rad(self) -> float:
        return math.radians(self.max_deg)

    @property
    def span_rad(self) -> float:
        return self.max_rad - self.min_rad


@dataclass(frozen=True, slots=True)
class GripperSpec:
    """The parallel-jaw gripper.

    Two fingers driven symmetrically by one controller. A ``MimicJoint`` is
    deliberately not used: it is a known integration trap, and two slides with
    a symmetric command are simpler to verify.
    """

    fingers: int
    axis: tuple[float, float, float]
    rest_offset: float
    min_opening: float
    max_opening: float
    max_velocity: float
    max_acceleration: float
    tip_length: float
    provenance: str


@dataclass(frozen=True, slots=True)
class SafetySpec:
    """Limits enforced by the safety supervisor (PLAN:irobots-v2:C09)."""

    workspace_centre: tuple[float, float]
    workspace_radius: float
    z_min: float
    z_max: float
    table_plane_z: float
    table_clearance: float
    self_collision_radius: float


@dataclass(frozen=True, slots=True)
class RobotModel:
    """Everything the rest of the system needs to know about the arm."""

    name: str
    base_height: Scalar
    upper_arm: Scalar
    fore_arm: Scalar
    wrist_post: Scalar
    gripper_link: Scalar
    grip_thickness: Scalar
    joints: Mapping[str, JointSpec]
    gripper: GripperSpec
    safety: SafetySpec
    home_pose: Mapping[str, float]
    servo_channels: Mapping[str, int]
    pca9685_address: int
    pulse_width_us: tuple[int, int]

    # -- derived geometry ---------------------------------------------------

    @property
    def wrist_to_tip(self) -> float:
        """Distance from the wrist joint to the gripper tip, along the tool axis."""
        return float(self.wrist_post) + float(self.gripper_link)

    @property
    def max_reach(self) -> float:
        """Maximum horizontal distance from the base axis to the tool tip."""
        return float(self.upper_arm) + float(self.fore_arm) + self.wrist_to_tip

    @property
    def min_reach(self) -> float:
        """Smallest horizontal distance the arm can fold to."""
        return max(0.0, float(self.upper_arm) - float(self.fore_arm) - self.wrist_to_tip)

    @property
    def max_height(self) -> float:
        """Height of the tool tip when the arm is fully extended upward."""
        return (
            float(self.base_height)
            + float(self.upper_arm)
            + float(self.fore_arm)
            + self.wrist_to_tip
        )

    def joint(self, name: str) -> JointSpec:
        try:
            return self.joints[name]
        except KeyError:
            raise ConfigError(f"unknown joint {name!r}") from None

    def joint_limits_rad(self) -> dict[str, tuple[float, float]]:
        return {n: (j.min_rad, j.max_rad) for n, j in self.joints.items()}

    def clamp_to_limits(self, pose: Mapping[str, float]) -> dict[str, float]:
        """Clamp every arm joint into its range. Gripper is left alone."""
        out: dict[str, float] = {}
        for name, value in pose.items():
            if name == GRIPPER_CHANNEL:
                out[name] = value
                continue
            spec = self.joint(name)
            out[name] = min(max(value, spec.min_rad), spec.max_rad)
        return out


def load_robot(path: str | Path = "config/robot.yaml") -> RobotModel:
    """Build a :class:`RobotModel` from ``config/robot.yaml``.

    Raises ``ConfigError`` on a missing key, a bad provenance tag, or an
    internally inconsistent model. Failing loudly at load time is the point:
    a half-specified robot that fails at grasp time is far more expensive.
    """
    raw = load_yaml(path)
    r = raw.get("robot")
    if not isinstance(r, dict):
        raise ConfigError("config/robot.yaml: missing top-level 'robot' mapping")

    links = r.get("links")
    if not isinstance(links, dict):
        raise ConfigError("config/robot.yaml: missing 'robot.links'")

    def link(name: str) -> Scalar:
        node = links.get(name)
        if not isinstance(node, dict):
            raise ConfigError(f"config/robot.yaml: missing 'robot.links.{name}'")
        return scalar_from(node, f"robot.links.{name}")

    joint_nodes = r.get("joints")
    if not isinstance(joint_nodes, list) or not joint_nodes:
        raise ConfigError("config/robot.yaml: 'robot.joints' must be a non-empty list")

    joints: dict[str, JointSpec] = {}
    for node in joint_nodes:
        name = str(node["name"])
        limits = node.get("limits")
        if not isinstance(limits, dict):
            raise ConfigError(f"config/robot.yaml: joint {name!r} has no 'limits'")
        axis = node.get("axis")
        origin = node.get("origin")
        if not (isinstance(axis, list) and len(axis) == 3):
            raise ConfigError(f"config/robot.yaml: joint {name!r} has a malformed 'axis'")
        if not (isinstance(origin, list) and len(origin) == 3):
            raise ConfigError(f"config/robot.yaml: joint {name!r} has a malformed 'origin'")
        joints[name] = JointSpec(
            name=name,
            axis=(float(axis[0]), float(axis[1]), float(axis[2])),
            parent=str(node.get("parent", "")),
            child=str(node.get("child", "")),
            origin=(float(origin[0]), float(origin[1]), float(origin[2])),
            min_deg=float(limits["min"]),
            max_deg=float(limits["max"]),
            max_velocity=float(node.get("max_velocity", 2.0)),
            max_acceleration=float(node.get("max_acceleration", 5.0)),
            provenance=str(limits.get("provenance", "design")),
        )

    missing = [j for j in JOINT_ORDER if j not in joints]
    if missing:
        raise ConfigError(f"config/robot.yaml: missing joints {missing}")

    g = r.get("gripper")
    if not isinstance(g, dict):
        raise ConfigError("config/robot.yaml: missing 'robot.gripper'")
    glimits = g.get("limits")
    if not isinstance(glimits, dict):
        raise ConfigError("config/robot.yaml: missing 'robot.gripper.limits'")
    gripper = GripperSpec(
        fingers=int(g.get("fingers", 2)),
        axis=(1.0, 0.0, 0.0),
        rest_offset=float(g.get("rest_offset", 0.0)),
        min_opening=float(glimits["min"]),
        max_opening=float(glimits["max"]),
        max_velocity=float(g.get("max_velocity", 0.05)),
        max_acceleration=float(g.get("max_acceleration", 0.2)),
        tip_length=float(g.get("tip_length", 0.039)),
        provenance=str(glimits.get("provenance", "design")),
    )

    s = r.get("safety")
    if not isinstance(s, dict):
        raise ConfigError("config/robot.yaml: missing 'robot.safety'")
    ws = s.get("workspace")
    if not isinstance(ws, dict):
        raise ConfigError("config/robot.yaml: missing 'robot.safety.workspace'")
    centre = ws.get("centre_xy")
    if not (isinstance(centre, list) and len(centre) == 2):
        raise ConfigError("config/robot.yaml: 'safety.workspace.centre_xy' must have 2 entries")
    safety = SafetySpec(
        workspace_centre=(float(centre[0]), float(centre[1])),
        workspace_radius=float(ws["radius"]),
        z_min=float(ws["z_min"]),
        z_max=float(ws["z_max"]),
        table_plane_z=float(s.get("table_plane_z", 0.0)),
        table_clearance=float(s.get("table_clearance", 0.002)),
        self_collision_radius=float(s.get("self_collision_radius", 0.06)),
    )

    servos = r.get("servos")
    if not isinstance(servos, dict):
        raise ConfigError("config/robot.yaml: missing 'robot.servos'")
    channels_node = servos.get("channels")
    if not isinstance(channels_node, dict):
        raise ConfigError("config/robot.yaml: missing 'robot.servos.channels'")
    pulse = servos.get("pulse_width_us")
    if not isinstance(pulse, dict):
        raise ConfigError("config/robot.yaml: missing 'robot.servos.pulse_width_us'")

    home = r.get("home_pose")
    if not isinstance(home, dict):
        raise ConfigError("config/robot.yaml: missing 'robot.home_pose'")

    model = RobotModel(
        name=str(r.get("name", "irobots_arm")),
        base_height=link("base_height"),
        upper_arm=link("upper_arm"),
        fore_arm=link("fore_arm"),
        wrist_post=link("wrist_post"),
        gripper_link=link("gripper_link"),
        grip_thickness=link("grip_thickness"),
        joints=joints,
        gripper=gripper,
        safety=safety,
        home_pose={k: float(v) for k, v in home.items()},
        servo_channels={str(k): int(v) for k, v in channels_node.items()},
        pca9685_address=int(servos.get("pca9685_address", 0x40)),
        pulse_width_us=(int(pulse["min"]), int(pulse["max"])),
    )

    _validate(model)
    return model


def _validate(model: RobotModel) -> None:
    """Internal consistency checks.

    These are the checks that catch a mis-specified robot while it is still
    cheap to fix, rather than after the arm has been built.
    """
    if model.max_reach <= 0.0:
        raise ConfigError("robot: max reach must be positive")
    if model.gripper.min_opening >= model.gripper.max_opening:
        raise ConfigError("robot: gripper min opening must be below max opening")
    if model.gripper.fingers != 2:
        raise ConfigError("robot: only a 2-finger parallel jaw is implemented")

    for name, joint in model.joints.items():
        if joint.min_deg >= joint.max_deg:
            raise ConfigError(f"robot: joint {name!r} has an empty range")
        if abs(sum(a * a for a in joint.axis) - 1.0) > 1e-6:
            raise ConfigError(f"robot: joint {name!r} axis is not a unit vector")

    # The workspace must lie inside the arm's reach, with margin. The legacy
    # 35x35cm square anchored at a base corner put its far corner 495mm away
    # against a 358mm arm: physically unreachable, and silently so.
    cx, cy = model.safety.workspace_centre
    far = math.hypot(cx, cy) + model.safety.workspace_radius
    if far > model.max_reach:
        raise ConfigError(
            f"robot: workspace is unreachable - far edge at {far:.3f}m but max reach "
            f"is {model.max_reach:.3f}m. Move the workspace or shorten it."
        )
    if model.safety.workspace_radius <= 0.0:
        raise ConfigError("robot: workspace radius must be positive")


def workspace_samples(model: RobotModel, n: int = 512) -> Sequence[tuple[float, float]]:
    """Deterministic sample points inside the workspace disk, for reach sweeps."""
    import numpy as np

    rng = np.random.default_rng(0)
    cx, cy = model.safety.workspace_centre
    radius = model.safety.workspace_radius
    r = radius * np.sqrt(rng.random(n))
    theta = rng.random(n) * 2.0 * math.pi
    return [(float(cx + ri * math.cos(t)), float(cy + ri * math.sin(t))) for ri, t in zip(r, theta, strict=True)]
