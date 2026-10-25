"""Trajectory driver node with per-skill deadline and e-stop (PLAN:irobots-v2:C25, I4, I6).

Converts a validated skill joint goal into a deterministic trapezoidal
trajectory and steps the simulator on ``use_sim_time``. Enforces the per-skill
deadline in the driver so an unreachable waypoint never hangs the episode.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from irobots_core.robot_model import RobotModel
from irobots_core.trajectory import plan_joint_trajectory
from irobots_msgs.messages import SkillResultMsg


class ControlNode:
    """Thin trajectory execution wrapper around irobots_core.trajectory (C25)."""

    def __init__(
        self,
        robot: RobotModel,
        sim: Any,
        *,
        use_sim_time: bool = True,
        default_deadline_s: float = 5.0,
    ) -> None:
        if not use_sim_time:
            raise ValueError("Invariant I6 requires use_sim_time=True on ControlNode")
        self.robot = robot
        self.sim = sim
        self.use_sim_time = use_sim_time
        self.default_deadline_s = default_deadline_s
        self.estopped: bool = False
        self.executed_count: int = 0

    def trigger_estop(self) -> None:
        """Halt trajectory execution and hold the current pose (D18)."""
        self.estopped = True
        self.sim.set_joint_targets(self.sim.joint_positions())

    def clear_estop(self) -> None:
        self.estopped = False

    def execute_joint_goal(
        self,
        *,
        skill: str,
        target_id: str,
        goal_pose: Mapping[str, float],
        opening: float,
        deadline_s: float | None = None,
    ) -> SkillResultMsg:
        """Execute a trajectory to ``goal_pose`` within ``deadline_s`` of sim time (T33)."""
        if self.estopped:
            return SkillResultMsg(
                skill=skill,
                target_id=target_id,
                approved=False,
                outcome="estop_aborted",
                elapsed_sim_s=0.0,
                timed_out=False,
            )

        budget_s = float(deadline_s if deadline_s is not None else self.default_deadline_s)
        start_pose = self.sim.joint_positions()
        traj = plan_joint_trajectory(self.robot, start_pose, goal_pose)

        if traj.duration > budget_s:
            # Step only up to the deadline and abort this skill cleanly (T33, I4)
            max_steps = max(1, int(budget_s / self.sim.dt))
            pt = traj.sample(budget_s)
            self.sim.set_joint_targets(pt.positions)
            self.sim.set_opening(opening)
            self.sim.step(max_steps)
            self.executed_count += 1
            return SkillResultMsg(
                skill=skill,
                target_id=target_id,
                approved=True,
                outcome="skill_timeout",
                elapsed_sim_s=max_steps * self.sim.dt,
                timed_out=True,
            )

        self.sim.set_joint_targets(goal_pose)
        self.sim.set_opening(opening)
        max_steps = max(1, int(budget_s / self.sim.dt))
        taken = self.sim.step_until_settled(max_steps=max_steps)
        elapsed = taken * self.sim.dt
        self.executed_count += 1
        timed_out = taken >= max_steps and elapsed >= budget_s - 1e-6
        return SkillResultMsg(
            skill=skill,
            target_id=target_id,
            approved=True,
            outcome="skill_timeout" if timed_out else "gripper_at_pose",
            elapsed_sim_s=elapsed,
            timed_out=timed_out,
        )
