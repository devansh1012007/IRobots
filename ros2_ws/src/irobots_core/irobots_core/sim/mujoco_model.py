"""MuJoCo model generation.

The MJCF is GENERATED from ``config/robot.yaml`` and ``config/scene.yaml``.
There is no hand-written XML in the repository: if the robot description
changes, the model changes with it, and the analytic kinematics and the
simulator cannot drift apart unnoticed.

Joint conventions (these must match ``irobots_core.kinematics`` exactly, or
the round-trip test fails):

* Pitch joints use the axis ``(0, -1, 0)`` in their parent frame, so a
  positive pitch raises the link.
* The tool body carries the constant 120-degree rotation
  ``quat="0.5 0.5 0.5 0.5"``, which puts the tool's Z (the approach axis)
  along the link and the tool's X (the jaw closing direction) along the arm
  plane's normal.
* ``wrist_yaw`` is at the tool tip and rotates about the tool's Z, i.e. the
  approach axis.

PLAN:irobots-v2:C21
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from ..robot_model import RobotModel

#: Solver settings tuned for small-object grasping.
#: - ``cone="elliptic"`` plus a high ``impratio`` makes the contact model
#:   far less prone to the "cube is squeezed out between the fingers" failure
#:   that a pyramidal cone produces.
#: - ``frictionloss`` is enabled on the finger pads so a grasped object does
#:   not creep.
#: - ``noslip_iterations`` gives the solver extra passes for the pinch.
DEFAULT_OPTIONS: Mapping[str, Any] = {
    "timestep": "0.002",
    "gravity": "0 0 -9.81",
    "integrator": "implicitfast",
    "cone": "elliptic",
    "impratio": "20",
    "noslip_iterations": "10",
    "iterations": "100",
    "solver": "Newton",
    "ls_iterations": "50",
}

#: Defaults applied to every geom, so the tuning is in one place.
DEFAULT_GEOM: Mapping[str, str] = {
    "condim": "3",
    "friction": "0.6 0.005 0.0001",
    "solref": "0.005 1",
    "solimp": "0.9 0.95 0.001",
    "rgba": "0.65 0.68 0.72 1",
}


def _f(v: float) -> str:
    return f"{v:.6g}"


def look_at_frame(position: Sequence[float], target: Sequence[float]) -> tuple[str, str]:
    """Build the ``xyaxes`` attribute for a MuJoCo camera at ``position``.

    MuJoCo cameras look along -Z with +X right and +Y up. Returns the
    ``xyaxes`` string.
    """
    pos = np.asarray(position, dtype=float)
    tgt = np.asarray(target, dtype=float)
    forward = tgt - pos
    norm = np.linalg.norm(forward)
    if norm < 1e-9:
        raise ValueError("camera position and target coincide")
    forward = forward / norm

    z_axis = -forward
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(z_axis, up))) > 0.999:
        # Looking straight up or down: pick a different reference.
        up = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(z_axis, up)
    x_norm = np.linalg.norm(x_axis)
    if x_norm < 1e-9:
        raise ValueError("degenerate camera frame")
    x_axis = x_axis / x_norm
    y_axis = np.cross(z_axis, x_axis)
    return (
        " ".join(_f(float(c)) for c in x_axis) + " " + " ".join(_f(float(c)) for c in y_axis),
        _f(float(z_axis[0])),
    )


def hfov_to_fovy(hfov_deg: float, width: int, height: int) -> float:
    """Convert a horizontal field of view to MuJoCo's vertical ``fovy``."""
    aspect = height / width
    return math.degrees(2.0 * math.atan(math.tan(math.radians(hfov_deg) / 2.0) * aspect))


@dataclass(frozen=True, slots=True)
class CameraSpec:
    """A camera in the scene."""

    name: str
    position: tuple[float, float, float]
    target: tuple[float, float, float]
    hfov_deg: float
    width: int
    height: int
    near: float
    far: float


def camera_from_scene(scene_cfg: Mapping[str, Any]) -> CameraSpec:
    """Read the camera specification out of ``config/scene.yaml``."""
    cam = scene_cfg.get("camera")
    if not isinstance(cam, dict):
        raise ValueError("scene config has no 'camera' mapping")
    position = tuple(float(c) for c in cam["position"])
    target = tuple(float(c) for c in cam["target"])
    return CameraSpec(
        name=str(cam.get("name", "camera")),
        position=position,  # type: ignore[arg-type]
        target=target,  # type: ignore[arg-type]
        hfov_deg=float(cam["hfov_deg"]),
        width=int(cam["width"]),
        height=int(cam["height"]),
        near=float(cam.get("near", 0.01)),
        far=float(cam.get("far", 5.0)),
    )


def build_mjcf(
    model: RobotModel,
    scene_cfg: Mapping[str, Any],
    *,
    objects: Sequence[Mapping[str, Any]] = (),
    options: Mapping[str, Any] | None = None,
) -> str:
    """Generate the MJCF for the arm, the table and the task objects.

    :param model: the robot description from ``config/robot.yaml``.
    :param scene_cfg: the parsed ``config/scene.yaml``.
    :param objects: extra free bodies to spawn, each with ``name``, ``size``
        and ``pos``.
    :param options: overrides for the solver settings.
    """
    opts = {**DEFAULT_OPTIONS, **(options or {})}
    table = scene_cfg["table"]
    base_pos = scene_cfg.get("base_position", [0.0, 0.0, 0.0])
    cam = camera_from_scene(scene_cfg)

    l1 = float(model.upper_arm)
    l2 = float(model.fore_arm)
    l3 = float(model.wrist_post)
    l4 = float(model.gripper_link)
    base_h = float(model.base_height)
    g = model.gripper
    # Half the pad thickness plus the minimum half-opening, so the pads'
    # inner faces sit at +/- (opening/2).
    pad_half = 0.004
    max_half = g.max_opening / 2.0
    slide_hi = max_half - pad_half
    # The pad plate extends back from the grasp point towards the housing.
    pad_len = 0.030
    # How far the on-axis housing stops short of the grasp point. It has to
    # clear the tallest object we pick, so that the housing never drives into
    # the top of an object while the jaws are closing on its sides.
    housing_clearance = 0.034

    j = model.joints
    yaw = j["base_yaw"]
    sh = j["shoulder_pitch"]
    el = j["elbow_pitch"]
    wp = j["wrist_pitch"]
    wy = j["wrist_yaw"]

    lines: list[str] = []
    add = lines.append
    add('<mujoco model="irobots_arm">')
    add('  <compiler angle="radian" coordinate="local" autolimits="true"/>')
    add("  <option " + " ".join(f'{k}="{v}"' for k, v in opts.items()) + ">")
    add('    <flag contact="enable" frictionloss="enable"/>')
    add("  </option>")
    add("  <default>")
    add(f'    <geom {DEFAULT_GEOM["condim"] and ""}type="capsule" size="0.010" '
        f'condim="{DEFAULT_GEOM["condim"]}" friction="{DEFAULT_GEOM["friction"]}" '
        f'solref="{DEFAULT_GEOM["solref"]}" solimp="{DEFAULT_GEOM["solimp"]}" '
        f'rgba="{DEFAULT_GEOM["rgba"]}"/>')
    add("  </default>")
    add("  <asset>")
    add('    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.5 0.6 0.75" '
        'rgb2="0.15 0.18 0.25" width="512" height="3072"/>')
    # A MATTE, plain surface. docs/hardware.md is explicit that the table
    # must be matte and unpatterned: a glossy or patterned table throws
    # specular highlights and reads as objects. It also keeps the geometric
    # detector's "what is the table colour" question well posed.
    add('    <material name="table_mat" rgba="0.38 0.39 0.42 1" specular="0.05" '
        'shininess="0.02" reflectance="0"/>')
    add("  </asset>")
    add("  <worldbody>")
    add('    <light name="sun" pos="0.3 0.2 1.2" dir="-0.3 -0.2 -1" directional="true" '
        'diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2" castshadow="true"/>')
    add('    <light name="fill" pos="-0.4 0.3 0.8" dir="0.4 -0.3 -0.8" directional="true" '
        'diffuse="0.35 0.35 0.35" specular="0.1 0.1 0.1" castshadow="false"/>')

    # --- table -----------------------------------------------------------
    t_size = [float(c) for c in table["size_xy"]]
    t_thick = float(table.get("thickness", 0.02))
    t_pos = [float(c) for c in table.get("position", [0.0, 0.0])]
    add(f'    <geom name="table" type="box" '
        f'pos="{_f(t_pos[0])} {_f(t_pos[1])} {_f(-t_thick / 2)}" '
        f'size="{_f(t_size[0] / 2)} {_f(t_size[1] / 2)} {_f(t_thick / 2)}" '
        f'material="table_mat" friction="0.8 0.005 0.0001" condim="3"/>')
    rim = table.get("rim") or {}
    if rim.get("enabled", False):
        rim_h = float(rim.get("height", 0.02))
        rim_t = float(rim.get("thickness", 0.01))
        hx = t_size[0] / 2.0
        hy = t_size[1] / 2.0
        for _sign, name, size, off in (
            (1, "rim_x_pos", (rim_t / 2, hy, rim_h / 2), (hx - rim_t / 2, 0)),
            (-1, "rim_x_neg", (rim_t / 2, hy, rim_h / 2), (-(hx - rim_t / 2), 0)),
            (1, "rim_y_pos", (hx, rim_t / 2, rim_h / 2), (0, hy - rim_t / 2)),
            (-1, "rim_y_neg", (hx, rim_t / 2, rim_h / 2), (0, -(hy - rim_t / 2))),
        ):
            add(f'    <geom name="{name}" type="box" '
                f'pos="{_f(t_pos[0] + off[0])} {_f(t_pos[1] + off[1])} {_f(rim_h / 2)}" '
                f'size="{_f(size[0])} {_f(size[1])} {_f(size[2])}" '
                f'rgba="0.45 0.45 0.5 1" friction="0.6 0.005 0.0001"/>')

    # --- camera ----------------------------------------------------------
    xyaxes, _ = look_at_frame(cam.position, cam.target)
    fovy = hfov_to_fovy(cam.hfov_deg, cam.width, cam.height)
    # MuJoCo's camera element has no near/far attributes; the clipping planes
    # live on the model and are set from `cam.near` / `cam.far` after the
    # model is compiled (see mujoco_sim.MuJoCoSim).
    add(f'    <camera name="{cam.name}" pos="{_f(cam.position[0])} {_f(cam.position[1])} '
        f'{_f(cam.position[2])}" xyaxes="{xyaxes}" fovy="{_f(fovy)}"/>')

    # --- task objects ----------------------------------------------------
    for obj in objects:
        obj_size = [float(c) for c in obj["size"]]
        obj_pos = [float(c) for c in obj["pos"]]
        mass = float(obj.get("mass", 0.04))
        rgba = obj.get("rgba", "0.9 0.3 0.3 1")
        add(f'    <body name="{obj["name"]}" pos="{_f(obj_pos[0])} {_f(obj_pos[1])} {_f(obj_pos[2])}">')
        add(f'      <freejoint name="{obj["name"]}_free"/>')
        add(f'      <geom name="{obj["name"]}_geom" type="box" '
            f'size="{_f(obj_size[0] / 2)} {_f(obj_size[1] / 2)} {_f(obj_size[2] / 2)}" '
            f'mass="{_f(mass)}" rgba="{rgba}" '
            f'friction="{obj.get("friction", 0.6)} 0.005 0.0001" condim="3" '
            f'solref="0.005 1" solimp="0.9 0.95 0.001"/>')
        add("    </body>")

    # --- the arm ---------------------------------------------------------
    # MJCF allows ONE joint per body, and a body's joint rotates that body
    # (and its children) about its own origin. So the chain is:
    #
    #   base (static, the column)
    #     yaw      - base_yaw about z, at the top of the column
    #       shoulder - shoulder_pitch, AT the shoulder pivot, carries the upper arm
    #         elbow  - elbow_pitch, at the elbow, carries the forearm
    #           wrist - wrist_pitch, at the wrist, carries the wrist link
    #             tool - the constant frame rotation, carries the gripper link
    #               fingers - wrist_yaw about the tool's Z (the approach axis)
    #                 finger_pos / finger_neg - the two slides
    #
    # Getting the joint/body pairing wrong is silent: the arm still compiles
    # and still moves, but the wrong link carries the rotation, so the
    # analytic and simulated kinematics disagree.
    bx, by, bz = (float(c) for c in base_pos)
    add(f'    <body name="base" pos="{_f(bx)} {_f(by)} {_f(bz)}">')
    add(f'      <geom name="base_column" type="cylinder" size="0.055 {_f(base_h / 2)}" '
        f'pos="0 0 {_f(base_h / 2)}" rgba="0.3 0.32 0.36 1" mass="2.0"/>')

    add(f'      <body name="yaw" pos="0 0 {_f(base_h)}">')
    add(f'        <joint name="base_yaw" type="hinge" axis="0 0 1" '
        f'range="{_f(yaw.min_rad)} {_f(yaw.max_rad)}" damping="1.0" armature="0.02"/>')
    # A body with no geom has no mass, and MuJoCo rejects a moving body with
    # zero inertia. The hub also gives the base yaw something to push against.
    add('        <geom name="yaw_hub" type="cylinder" size="0.022 0.008" '
        'pos="0 0 0.008" rgba="0.35 0.37 0.42 1" mass="0.08"/>')

    add('        <body name="shoulder" pos="0 0 0">')
    add(f'          <joint name="shoulder_pitch" type="hinge" axis="0 -1 0" '
        f'range="{_f(sh.min_rad)} {_f(sh.max_rad)}" damping="1.0" armature="0.02"/>')
    add(f'          <geom name="upper_arm_geom" fromto="0 0 0 {_f(l1)} 0 0" size="0.012" '
        f'rgba="0.75 0.78 0.82 1" mass="0.35"/>')

    add(f'          <body name="elbow" pos="{_f(l1)} 0 0">')
    add(f'            <joint name="elbow_pitch" type="hinge" axis="0 -1 0" '
        f'range="{_f(el.min_rad)} {_f(el.max_rad)}" damping="0.8" armature="0.015"/>')
    add(f'            <geom name="fore_arm_geom" fromto="0 0 0 {_f(l2)} 0 0" size="0.011" '
        f'rgba="0.72 0.75 0.80 1" mass="0.25"/>')

    add(f'            <body name="wrist" pos="{_f(l2)} 0 0">')
    add(f'              <joint name="wrist_pitch" type="hinge" axis="0 -1 0" '
        f'range="{_f(wp.min_rad)} {_f(wp.max_rad)}" damping="0.6" armature="0.01"/>')
    add(f'              <geom name="wrist_geom" fromto="0 0 0 {_f(l3)} 0 0" size="0.010" '
        f'rgba="0.70 0.73 0.78 1" mass="0.12"/>')

    # The tool body: the constant 120-degree rotation puts the tool's Z along
    # the link (the approach axis) and the tool's X along the arm plane's
    # normal (the jaw closing direction).
    #
    # The GRASP POINT is the tool's z = L4 end (the "fingers" body origin).
    # Everything that sits ON the approach axis has to stop short of it: the
    # grasp point is where the object is, so an on-axis housing that reaches
    # the grasp point would drive straight into the top of the object and
    # stop the gripper from descending to the object's middle. The housing
    # therefore stops HOUSING_CLEARANCE above the grasp point, and the finger
    # pads reach back to meet it. The pads are off-axis (at +/- opening/2), so
    # they straddle the object and never touch it.
    add(f'              <body name="tool" pos="{_f(l3)} 0 0" quat="0.5 0.5 0.5 0.5">')
    add(f'                <geom name="gripper_link_geom" '
        f'fromto="0 0 0 0 0 {_f(l4 - housing_clearance)}" size="0.010" '
        f'rgba="0.65 0.68 0.74 1" mass="0.10"/>')

    # The tip, and the wrist yaw about the approach axis.
    add(f'                <body name="fingers" pos="0 0 {_f(l4)}">')
    add(f'                  <joint name="wrist_yaw" type="hinge" axis="0 0 1" '
        f'range="{_f(wy.min_rad)} {_f(wy.max_rad)}" damping="0.4" armature="0.008"/>')
    # An explicit inertial spec rather than a collision geom: a body with no
    # geom has no mass, and MuJoCo rejects a moving body with zero inertia.
    # A collision sphere here would sit exactly on the grasp point and hit
    # every object the gripper tries to pick up.
    add('                  <inertial pos="0 0 0" mass="0.01" '
        'diaginertia="1e-6 1e-6 1e-6"/>')

    # Two prismatic fingers driven symmetrically from ONE opening command.
    # No mimic joint: that is a known integration trap.
    #
    # The pad's inner face must land exactly at +/- (opening/2). The pad's
    # centre is therefore 2*pad_half out from the body origin: the inner face
    # is then at body x = 2*pad_half - pad_half = pad_half, and the body sits
    # at ctrl = (opening/2) - pad_half, giving an inner face at opening/2.
    # Getting this wrong makes the jaws 2*pad_half too narrow, which for a
    # 40mm cube means the gripper shoves the object away instead of
    # straddling it.
    #
    # The pads extend BACK from the grasp point (the tool's -Z is up) so they
    # reach the housing while their front faces stay at the grasp point.
    for sign, name in ((1, "finger_pos"), (-1, "finger_neg")):
        add(f'                  <body name="{name}" pos="0 0 0">')
        add(f'                    <joint name="{name}_slide" type="slide" axis="1 0 0" '
            f'range="{_f(-slide_hi)} {_f(slide_hi)}" damping="4.0" armature="0.001"/>')
        add(f'                    <geom name="{name}_pad" type="box" '
            f'pos="{_f(sign * 2.0 * pad_half)} 0 {_f(-pad_len / 2)}" '
            f'size="0.004 0.018 {_f(pad_len / 2)}" '
            f'rgba="0.2 0.2 0.22 1" mass="0.02" '
            f'friction="1.4 0.01 0.0001" condim="4" '
            f'solref="0.003 1" solimp="0.95 0.99 0.001"/>')
        add("                  </body>")
    add("                </body>")
    add("              </body>")
    add("            </body>")
    add("          </body>")
    add("        </body>")
    add("      </body>")
    add("    </body>")
    add("  </worldbody>")

    # --- contact exclusions -----------------------------------------------
    # Adjacent links' capsules overlap at the joints by construction (a
    # capsule of radius 12mm meets one of radius 11mm at a shared origin).
    # Without these exclusions the arm fights itself at t=0: the penetrating
    # contact between the base column and the upper arm produces a torque
    # that holds the base yaw at zero no matter what the actuator commands.
    add("  <contact>")
    for parent, child in (
        ("base", "yaw"),
        ("base", "shoulder"),
        # The yaw hub is coaxial with the shoulder pivot, so the upper arm's
        # start sphere always overlaps it. Without this exclusion the
        # penetrating contact fights the shoulder pitch and the arm simply
        # stops short of every commanded pose.
        ("yaw", "shoulder"),
        ("shoulder", "elbow"),
        ("elbow", "wrist"),
        ("wrist", "tool"),
        ("tool", "fingers"),
        # The gripper link's capsule runs past the finger pads.
        ("tool", "finger_pos"),
        ("tool", "finger_neg"),
        ("fingers", "finger_pos"),
        ("fingers", "finger_neg"),
        # The pads must not fight each other: the jaw's minimum opening is
        # enforced by the slide joint's range, not by a contact.
        ("finger_pos", "finger_neg"),
    ):
        add(f'    <exclude body1="{parent}" body2="{child}"/>')
    add("  </contact>")

    # --- actuators -------------------------------------------------------
    add("  <actuator>")
    for name, joint, kp, force in (
        ("base_yaw", "base_yaw", 40.0, 6.0),
        ("shoulder_pitch", "shoulder_pitch", 60.0, 12.0),
        ("elbow_pitch", "elbow_pitch", 40.0, 8.0),
        ("wrist_pitch", "wrist_pitch", 20.0, 4.0),
        ("wrist_yaw", "wrist_yaw", 12.0, 2.5),
    ):
        spec = model.joints[joint]
        add(f'    <position name="{name}" joint="{joint}" kp="{_f(kp)}" kv="{_f(kp * 0.1)}" '
            f'forcerange="{_f(-force)} {_f(force)}" '
            f'ctrlrange="{_f(spec.min_rad)} {_f(spec.max_rad)}"/>')
    # The fingers: position control with a force limit, so the jaws squeeze
    # with a BOUNDED force. This is the force control the red team said was
    # missing: without it a position-controlled jaw drives into the object
    # with unlimited force and ejects it.
    add(f'    <position name="finger_pos" joint="finger_pos_slide" kp="400" kv="20" '
        f'forcerange="-1.5 1.5" ctrlrange="{_f(-slide_hi)} {_f(slide_hi)}"/>')
    add(f'    <position name="finger_neg" joint="finger_neg_slide" kp="400" kv="20" '
        f'forcerange="-1.5 1.5" ctrlrange="{_f(-slide_hi)} {_f(slide_hi)}"/>')
    add("  </actuator>")
    add("</mujoco>")
    return "\n".join(lines)
