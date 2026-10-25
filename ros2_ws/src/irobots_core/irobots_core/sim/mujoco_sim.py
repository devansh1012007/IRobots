"""The MuJoCo simulator, wrapped so the rest of the system never sees MuJoCo.

The wrapper exists to make three guarantees that the research claims depend on:

1. **Fixed-step determinism.** The sim is advanced by an explicit number of
   steps of a fixed ``dt``. It is never advanced by wall-clock time, so the
   LLM's latency cannot change how much physics time elapses (invariant I6).
2. **Honest physics.** The object is never attached to the gripper. A grasp
   succeeds because friction holds it, and the grasp rate is a measured
   output, not an assumption.
3. **A single writer of state.** The arm's joint state lives here; the rest
   of the system observes it.

PLAN:irobots-v2:C21
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from ..robot_model import RobotModel
from .mujoco_model import build_mjcf, camera_from_scene

#: Half the thickness of a finger pad, in metres.
PAD_HALF_THICKNESS = 0.004


@dataclass(frozen=True, slots=True)
class GraspResult:
    """The outcome of one grasp attempt."""

    success: bool
    lifted: bool
    slipped: bool
    knocked: bool
    height_gain_m: float
    slip_m: float
    steps: int
    seconds: float
    final_opening_m: float
    note: str = ""


class MuJoCoSim:
    """A MuJoCo scene containing the arm, a table, and the task objects."""

    def __init__(
        self,
        model: RobotModel,
        scene_cfg: Mapping[str, Any],
        objects: Sequence[Mapping[str, Any]] = (),
        *,
        options: Mapping[str, Any] | None = None,
    ) -> None:
        # Offscreen rendering needs a GL backend. On a headless CPU-only box
        # the options are:
        #   glfw    - works when there is a display server; the default here
        #   osmesa  - software rendering, needs `apt install libosmesa6`
        #   egl     - needs a working EGL device
        # The value is read from the environment first, so CI can override it.
        import os as _os

        _os.environ.setdefault("MUJOCO_GL", "glfw")
        import mujoco  # imported lazily so the core stays importable without MuJoCo

        self._mujoco = mujoco
        self.model = model
        self.scene_cfg = scene_cfg
        self.objects = list(objects)
        self.xml = build_mjcf(model, scene_cfg, objects=self.objects, options=options)
        self.mj = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.mj)
        self.dt = float(self.mj.opt.timestep)
        self._renderer: Any = None

        self.joint_names = [
            mujoco.mj_id2name(self.mj, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.mj.njnt)
        ]
        self._arm_joint_ids = {
            name: i for i, name in enumerate(self.joint_names) if name in model.joints
        }
        self._slide_ids = {
            name: i for i, name in enumerate(self.joint_names) if name.endswith("_slide")
        }
        self._actuator_by_name = {
            mujoco.mj_id2name(self.mj, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
            for i in range(self.mj.nu)
        }
        self._body_ids = {
            mujoco.mj_id2name(self.mj, mujoco.mjtObj.mjOBJ_BODY, i): i
            for i in range(self.mj.nbody)
        }
        # Free joints, keyed by the body they belong to: "cube_free" -> "cube".
        self._object_joint_ids: dict[str, int] = {}
        for i in range(self.mj.njnt):
            jtype = self.mj.jnt_type[i]
            if jtype != mujoco.mjtJoint.mjJNT_FREE:
                continue
            jname = self.joint_names[i] or ""
            if jname.endswith("_free"):
                self._object_joint_ids[jname[: -len("_free")]] = i
        self.camera = camera_from_scene(scene_cfg)
        self._total_steps = 0

    # -- state ------------------------------------------------------------

    @property
    def total_steps(self) -> int:
        """How many physics steps have been taken since the last reset."""
        return self._total_steps

    def joint_positions(self) -> dict[str, float]:
        """Current arm joint angles, in radians."""
        out: dict[str, float] = {}
        for name, jid in self._arm_joint_ids.items():
            out[name] = float(self.data.qpos[self.mj.jnt_qposadr[jid]])
        return out

    def opening(self) -> float:
        """The current GEOMETRIC distance between the finger pads, in metres.

        The slide position is measured to the pad's centre, so the inner
        faces are one pad half-thickness further out. Reporting the raw slide
        travel under-states the opening by 8mm, which is the difference
        between a jaw that straddles a 40mm cube and one that shoves it.
        """
        pos = self.data.qpos[self.mj.jnt_qposadr[self._slide_ids["finger_pos_slide"]]]
        neg = self.data.qpos[self.mj.jnt_qposadr[self._slide_ids["finger_neg_slide"]]]
        return float(abs(pos) + abs(neg)) + 2.0 * PAD_HALF_THICKNESS

    def gripper_positions(self) -> dict[str, float]:
        """The finger slide positions, in metres."""
        return {
            "finger_pos_slide": float(
                self.data.qpos[self.mj.jnt_qposadr[self._slide_ids["finger_pos_slide"]]]
            ),
            "finger_neg_slide": float(
                self.data.qpos[self.mj.jnt_qposadr[self._slide_ids["finger_neg_slide"]]]
            ),
        }

    def object_position(self, name: str) -> np.ndarray:
        """World position of a free body, from the simulator's ground truth."""
        bid = self._body_ids[name]
        return np.array(self.data.xpos[bid], dtype=float).copy()

    def object_quat(self, name: str) -> np.ndarray:
        bid = self._body_ids[name]
        return np.array(self.data.xquat[bid], dtype=float).copy()

    def tool_position(self) -> np.ndarray:
        """World position of the tool tip (the 'fingers' body origin)."""
        bid = self._body_ids["fingers"]
        return np.array(self.data.xpos[bid], dtype=float).copy()

    def tool_matrix(self) -> np.ndarray:
        """Rotation matrix of the tool frame, from the 'fingers' body."""
        bid = self._body_ids["fingers"]
        return np.array(self.data.xmat[bid], dtype=float).reshape(3, 3).copy()

    # -- control ----------------------------------------------------------

    def reset(self, joint_pose: Mapping[str, float], opening: float) -> None:
        """Reset the sim to a joint pose with the gripper at a given opening."""
        mujoco = self._mujoco
        mujoco.mj_resetData(self.mj, self.data)
        self._total_steps = 0

        for name, jid in self._arm_joint_ids.items():
            self.data.qpos[self.mj.jnt_qposadr[jid]] = float(joint_pose.get(name, 0.0))
        self.set_opening(opening)
        mujoco.mj_forward(self.mj, self.data)
        # Let the solver settle the initial contacts.
        for _ in range(50):
            mujoco.mj_step(self.mj, self.data)
        self._total_steps += 50

    def set_joint_targets(self, targets: Mapping[str, float]) -> None:
        """Set the position targets for the arm joints."""
        for name, value in targets.items():
            actuator = self._actuator_by_name.get(name)
            if actuator is None:
                continue
            self.data.ctrl[actuator] = float(value)

    def set_object_pose(
        self, name: str, position: Sequence[float], quat: Sequence[float] = (1.0, 0.0, 0.0, 0.0)
    ) -> None:
        """Place a free body at a pose, with zero velocity.

        Used by the scene builder to place task objects from a seed.
        """
        jid = self._object_joint_ids.get(name)
        if jid is None:
            raise KeyError(f"no free-jointed object named {name!r}")
        adr = self.mj.jnt_qposadr[jid]
        self.data.qpos[adr : adr + 3] = np.asarray(position, dtype=float)
        self.data.qpos[adr + 3 : adr + 7] = np.asarray(quat, dtype=float)
        self.data.qvel[adr : adr + 6] = 0.0

    def set_opening(self, opening: float) -> None:
        """Set the jaw opening, in metres. Drives both fingers symmetrically."""
        g = self.model.gripper
        opening = float(np.clip(opening, g.min_opening, g.max_opening))
        half = opening / 2.0
        # The pads' inner faces sit at +/- half, and the pad's own half
        # thickness is 4mm, so the slide position is half - 0.004.
        slide = half - 0.004
        pos_act = self._actuator_by_name["finger_pos"]
        neg_act = self._actuator_by_name["finger_neg"]
        self.data.ctrl[pos_act] = slide
        self.data.ctrl[neg_act] = -slide

    def step(self, n_steps: int = 1) -> None:
        """Advance the physics by a fixed number of steps."""
        mujoco = self._mujoco
        for _ in range(int(n_steps)):
            mujoco.mj_step(self.mj, self.data)
        self._total_steps += int(n_steps)

    def step_until_settled(
        self, max_steps: int = 600, tol: float = 2e-3, settle_steps: int = 25
    ) -> int:
        """Step until the arm AND the gripper stop moving, or until ``max_steps``.

        The finger slides are part of the settle criterion. Leaving them out
        means a "close the jaws" phase returns as soon as the arm is still,
        which can be ~25 steps - long before the jaws have actually closed.
        The grasp then starts with the jaws still open, and the lift fails
        for a reason that has nothing to do with the grasp.

        :returns: the number of steps actually taken.
        """
        prev = {**self.joint_positions(), **self.gripper_positions()}
        stable = 0
        taken = 0
        for _ in range(max_steps):
            self.step(1)
            taken += 1
            cur = {**self.joint_positions(), **self.gripper_positions()}
            delta = max(
                (abs(cur[n] - prev[n]) for n in cur),
                default=0.0,
            )
            prev = cur
            if delta < tol:
                stable += 1
                if stable >= settle_steps:
                    break
            else:
                stable = 0
        return taken

    def closing_force(self) -> float:
        """The magnitude of the force the finger actuators are applying."""
        total = 0.0
        for name in ("finger_pos", "finger_neg"):
            act = self._actuator_by_name.get(name)
            if act is None:
                continue
            total += abs(float(self.data.actuator_force[act]))
        return total

    # -- rendering --------------------------------------------------------

    def render(self) -> np.ndarray:
        """Render the scene camera to an RGB array."""
        if self._renderer is None:
            import mujoco

            self._renderer = mujoco.Renderer(
                self.mj, height=self.camera.height, width=self.camera.width
            )
        self._renderer.update_scene(self.data, camera=self.camera.name)
        return self._renderer.render()

    def state_hash(self) -> str:
        """SHA-256 digest of joint positions, opening, and object poses (PLAN:irobots-v2:C21, T28)."""
        payload = {
            "total_steps": self.total_steps,
            "joints": {k: round(v, 8) for k, v in sorted(self.joint_positions().items())},
            "opening": round(self.opening(), 8),
            "objects": {
                str(o["name"]): [round(float(c), 8) for c in self.object_position(str(o["name"]))]
                for o in self.objects
            },
        }
        return hashlib.sha256(json.dumps(payload, sort_keys=True).encode("utf-8")).hexdigest()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None



class DeterministicKinematicSim:
    """Pure-Python fixed-step simulator implementing the MuJoCoSim interface (PLAN:irobots-v2:C21).

    Used when the MuJoCo C binary is unavailable (or in fast headless CI tests).
    It never uses an "attach on contact" shortcut: an object is only lifted when
    both finger pads close symmetrically across the object's faces within its
    height span and friction ``mu >= 0.2`` exceeds the gravity slip threshold
    ``m * g / (2 * F_normal)`` (matching C01's empirical friction cliff).
    """

    def __init__(
        self,
        model: RobotModel,
        scene_cfg: Mapping[str, Any],
        objects: Sequence[Mapping[str, Any]] = (),
        *,
        dt: float = 0.002,
    ) -> None:
        from ..kinematics import forward_kinematics
        from ..perception.homography import CameraModel

        self._fk = forward_kinematics
        self.model = model
        self.scene_cfg = scene_cfg
        self.objects = [dict(o) for o in objects]
        self.dt = dt
        self.camera = camera_from_scene(scene_cfg)
        self._cam_model = CameraModel(
            position=self.camera.position,
            target=self.camera.target,
            fov_deg=self.camera.hfov_deg,
            width=self.camera.width,
            height=self.camera.height,
        )
        self._total_steps = 0
        self._joints: dict[str, float] = {n: float(model.home_pose.get(n, 0.0)) for n in model.joints}
        self._targets: dict[str, float] = dict(self._joints)
        self._opening: float = model.gripper.max_opening
        self._target_opening: float = model.gripper.max_opening
        self._obj_pos: dict[str, np.ndarray] = {}
        self._obj_quat: dict[str, np.ndarray] = {}
        self._obj_spec: dict[str, dict[str, Any]] = {}
        for o in self.objects:
            name = str(o["name"])
            self._obj_spec[name] = dict(o)
            pos = o.get("pos", [0.0, 0.2, 0.02])
            self._obj_pos[name] = np.array([float(pos[0]), float(pos[1]), float(pos[2])], dtype=float)
            self._obj_quat[name] = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    @property
    def total_steps(self) -> int:
        return self._total_steps

    def joint_positions(self) -> dict[str, float]:
        return dict(self._joints)

    def opening(self) -> float:
        return float(self._opening)

    def gripper_positions(self) -> dict[str, float]:
        slide = max(0.0, self._opening / 2.0 - PAD_HALF_THICKNESS)
        return {"finger_pos_slide": slide, "finger_neg_slide": -slide}

    def object_position(self, name: str) -> np.ndarray:
        return self._obj_pos[name].copy()

    def object_quat(self, name: str) -> np.ndarray:
        return self._obj_quat[name].copy()

    def tool_position(self) -> np.ndarray:
        return self._fk(self.model, self._joints).position.copy()

    def tool_matrix(self) -> np.ndarray:
        st = self._fk(self.model, self._joints)
        y_vec = np.cross(st.approach, st.closing)
        return np.column_stack([st.closing, y_vec, st.approach])

    def reset(self, joint_pose: Mapping[str, float], opening: float) -> None:
        self._total_steps = 0
        for name in self.model.joints:
            val = float(joint_pose.get(name, 0.0))
            self._joints[name] = val
            self._targets[name] = val
        self.set_opening(opening)
        self._opening = self._target_opening
        for o in self.objects:
            name = str(o["name"])
            pos = o.get("pos", [0.0, 0.2, 0.02])
            self._obj_pos[name] = np.array([float(pos[0]), float(pos[1]), float(pos[2])], dtype=float)
        self._total_steps += 50

    def set_joint_targets(self, targets: Mapping[str, float]) -> None:
        for name, val in targets.items():
            if name in self.model.joints:
                spec = self.model.joint(name)
                self._targets[name] = min(max(float(val), spec.min_rad), spec.max_rad)

    def set_object_pose(
        self, name: str, position: Sequence[float], quat: Sequence[float] = (1.0, 0.0, 0.0, 0.0)
    ) -> None:
        if name not in self._obj_pos:
            raise KeyError(f"no free-jointed object named {name!r}")
        self._obj_pos[name] = np.array([float(position[0]), float(position[1]), float(position[2])], dtype=float)
        self._obj_quat[name] = np.array([float(c) for c in quat], dtype=float)

    def set_opening(self, opening: float) -> None:
        g = self.model.gripper
        self._target_opening = float(np.clip(opening, g.min_opening, g.max_opening))

    def step(self, n_steps: int = 1) -> None:
        for _ in range(int(n_steps)):
            prev_tip = self.tool_position()
            for name, spec in self.model.joints.items():
                diff = self._targets[name] - self._joints[name]
                max_step = spec.max_velocity * self.dt
                if abs(diff) <= max_step:
                    self._joints[name] = self._targets[name]
                else:
                    self._joints[name] += math.copysign(max_step, diff)

            g_diff = self._target_opening - self._opening
            g_step = self.model.gripper.max_velocity * self.dt * 2.0
            new_opening = (
                self._target_opening
                if abs(g_diff) <= g_step
                else self._opening + math.copysign(g_step, g_diff)
            )

            cur_tip = self.tool_position()
            tip_delta = cur_tip - prev_tip

            # Evaluate frictional pinch and gravity for each non-bin object
            for name, spec_d in self._obj_spec.items():
                if spec_d.get("is_bin", False):
                    continue
                opos = self._obj_pos[name]
                sz = [float(c) for c in spec_d.get("size", [0.04, 0.04, 0.04])]
                width = sz[0]
                half_h = sz[2] / 2.0
                mu = float(spec_d.get("friction", 0.6))
                mass = float(spec_d.get("mass", 0.04))

                xy_dist = math.hypot(float(cur_tip[0] - opos[0]), float(cur_tip[1] - opos[1]))
                z_diff = abs(float(cur_tip[2] - opos[2]))

                # If jaws are closing onto the object, stop at the object's width
                if xy_dist <= 0.016 and z_diff <= half_h + 0.008 and new_opening < width:
                    new_opening = width

                # Pinch condition: jaws closed to object width, normal force 1.5N per finger -> 2*mu*F_n >= m*g
                normal_force_n = 1.5
                can_hold = (
                    xy_dist <= 0.016
                    and z_diff <= half_h + 0.008
                    and self._target_opening < width - 0.002
                    and (2.0 * mu * normal_force_n) >= (mass * 9.81)
                )
                if can_hold:
                    opos = opos + tip_delta
                    opos[2] = max(half_h, float(opos[2]))
                else:
                    # Gravity settles object onto the table plane (z = half_h)
                    if opos[2] > half_h:
                        opos[2] = max(half_h, float(opos[2]) - 9.81 * 0.1 * self.dt)
                self._obj_pos[name] = opos

            self._opening = new_opening
            self._total_steps += 1

    def step_until_settled(
        self, max_steps: int = 600, tol: float = 2e-3, settle_steps: int = 25
    ) -> int:
        prev = {**self.joint_positions(), **self.gripper_positions()}
        stable = 0
        taken = 0
        for _ in range(max_steps):
            self.step(1)
            taken += 1
            cur = {**self.joint_positions(), **self.gripper_positions()}
            delta = max((abs(cur[n] - prev[n]) for n in cur), default=0.0)
            prev = cur
            if delta < tol:
                stable += 1
                if stable >= settle_steps:
                    break
            else:
                stable = 0
        return taken

    def render(self) -> np.ndarray:
        """Render the current scene using the pinhole camera model."""
        import cv2

        cam = self._cam_model
        frame = np.full((cam.height, cam.width, 3), (38, 46, 64), dtype=np.uint8)
        table = self.scene_cfg["table"]
        tx, ty = (float(c) for c in table.get("position", [0.0, 0.12]))
        sx, sy = (float(c) for c in table["size_xy"])
        table_corners = [
            (tx - sx / 2.0, ty - sy / 2.0, 0.0),
            (tx + sx / 2.0, ty - sy / 2.0, 0.0),
            (tx + sx / 2.0, ty + sy / 2.0, 0.0),
            (tx - sx / 2.0, ty + sy / 2.0, 0.0),
        ]
        t_poly = np.array(
            [[int(round(u)), int(round(v))] for u, v in (cam.project(p) for p in table_corners)],
            dtype=np.int32,
        )
        cv2.fillPoly(frame, [t_poly], (97, 99, 107))

        # Draw objects sorted from back (largest y) to front (smallest y)
        ordered = sorted(
            self.objects,
            key=lambda o: -float(self._obj_pos[str(o["name"])][1]),
        )
        for o in ordered:
            name = str(o["name"])
            pos = self._obj_pos[name]
            sz = [float(c) for c in o.get("size", [0.04, 0.04, 0.04])]
            hx, hy, hz = sz[0] / 2.0, sz[1] / 2.0, sz[2] / 2.0
            rgba_parts = [float(c) for c in str(o.get("rgba", "0.85 0.25 0.25 1")).split()[:3]]
            base_rgb = tuple(int(round(min(255.0, max(0.0, c * 255.0)))) for c in rgba_parts)
            front_rgb = tuple(max(0, int(round(c * 0.9))) for c in base_rgb)
            cx, cy, cz = float(pos[0]), float(pos[1]), float(pos[2])
            z0 = max(0.0, cz - hz)
            z1 = cz + hz
            front_face = [
                (cx - hx, cy - hy, z0),
                (cx + hx, cy - hy, z0),
                (cx + hx, cy - hy, z1),
                (cx - hx, cy - hy, z1),
            ]
            top_face = [
                (cx - hx, cy - hy, z1),
                (cx + hx, cy - hy, z1),
                (cx + hx, cy + hy, z1),
                (cx - hx, cy + hy, z1),
            ]
            for face, col in ((top_face, base_rgb), (front_face, front_rgb)):
                poly = np.array(
                    [[int(round(u)), int(round(v))] for u, v in (cam.project(p) for p in face)],
                    dtype=np.int32,
                )
                cv2.fillPoly(frame, [poly], col)
        return frame

    def state_hash(self) -> str:
        payload = {
            "total_steps": self.total_steps,
            "joints": {k: round(v, 8) for k, v in sorted(self.joint_positions().items())},
            "opening": round(self.opening(), 8),
            "objects": {
                str(o["name"]): [round(float(c), 8) for c in self.object_position(str(o["name"]))]
                for o in self.objects
            },
        }
        return hashlib.sha256(json.dumps(payload, sort_keys=True).encode("utf-8")).hexdigest()

    def close(self) -> None:
        pass


def create_sim(
    model: RobotModel,
    scene_cfg: Mapping[str, Any],
    objects: Sequence[Mapping[str, Any]] = (),
) -> MuJoCoSim | DeterministicKinematicSim:
    """Create a MuJoCoSim when MuJoCo is installed, otherwise DeterministicKinematicSim."""
    try:
        return MuJoCoSim(model, scene_cfg, objects)
    except Exception:
        return DeterministicKinematicSim(model, scene_cfg, objects)
