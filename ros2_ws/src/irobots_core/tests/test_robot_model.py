"""Tests for the robot model: the single source of truth (PLAN:irobots-v2:C06).

The robot is not physically assembled, so these tests are the guard rail on
the *specification*: if a link length drifts away from the STL measurement it
came from, or a constant loses its provenance tag, the specification stops
being auditable and every downstream claim becomes unverifiable.
"""

from __future__ import annotations

import math

import pytest

from irobots_core.config import ConfigError
from irobots_core.robot_model import JOINT_ORDER, load_robot

#: Link lengths measured from the STL bounding boxes in
#: robotic arm schematics/5-dof-robotic-arm-5.snapshot.3.zip.
#: Arm 1 = 194mm, Arm 2 = 130mm, Arm 3 = 34mm.
STL_MEASURED_LINKS_M = {
    "upper_arm": 0.194,
    "fore_arm": 0.130,
    "wrist_post": 0.034,
}


def test_link_lengths_match_the_stl_measurements(robot) -> None:
    """T01: the geometry still matches the meshes it was derived from."""
    for name, expected in STL_MEASURED_LINKS_M.items():
        actual = float(getattr(robot, name))
        assert actual == pytest.approx(expected, abs=1e-9), (
            f"{name} drifted from its STL measurement: {actual} != {expected}"
        )


def test_every_constant_carries_a_provenance_tag(robot) -> None:
    """T02: nothing physical is specified without saying where it came from."""
    for name in ("base_height", "upper_arm", "fore_arm", "wrist_post", "gripper_link"):
        scalar = getattr(robot, name)
        assert scalar.provenance in {"measured", "design", "assumed"}, (
            f"{name} has no valid provenance tag"
        )
    for joint in robot.joints.values():
        assert joint.provenance in {"measured", "design", "assumed"}


def test_all_five_joints_are_present(robot) -> None:
    assert set(JOINT_ORDER) == set(robot.joints)
    assert len(robot.joints) == 5


def test_joint_limits_are_in_degrees_and_ordered(robot) -> None:
    """A limits table read in the wrong unit silently disarms the arm."""
    for name, joint in robot.joints.items():
        assert joint.min_deg < joint.max_deg, f"{name} has an empty range"
        assert joint.min_deg > -360.0 and joint.max_deg < 360.0, (
            f"{name} limits look like radians, not degrees"
        )
        assert joint.min_rad == pytest.approx(math.radians(joint.min_deg))


def test_joint_axes_are_unit_vectors(robot) -> None:
    for name, joint in robot.joints.items():
        length = math.sqrt(sum(a * a for a in joint.axis))
        assert length == pytest.approx(1.0, abs=1e-6), f"{name} axis is not a unit vector"


def test_workspace_is_inside_the_reach(robot) -> None:
    """The legacy 35x35cm square was 495mm from the base against a 358mm arm."""
    cx, cy = robot.safety.workspace_centre
    far = math.hypot(cx, cy) + robot.safety.workspace_radius
    assert far <= robot.max_reach, (
        f"the workspace reaches {far:.3f}m but the arm only reaches {robot.max_reach:.3f}m"
    )


def test_a_missing_config_file_fails_loudly() -> None:
    with pytest.raises(ConfigError):
        load_robot("config/does_not_exist.yaml")


def test_a_workspace_that_cannot_be_reached_is_rejected() -> None:
    """A configuration error must fail at load, not silently at grasp time."""
    import copy

    from irobots_core.robot_model import _validate

    model = load_robot()
    broken = copy.copy(model)
    object.__setattr__(
        broken,
        "safety",
        type(model.safety)(
            workspace_centre=(0.0, 0.9),
            workspace_radius=0.5,
            z_min=0.0,
            z_max=0.3,
            table_plane_z=0.0,
            table_clearance=0.002,
            self_collision_radius=0.06,
        ),
    )
    with pytest.raises(ConfigError, match="unreachable"):
        _validate(broken)
