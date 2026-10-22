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

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
