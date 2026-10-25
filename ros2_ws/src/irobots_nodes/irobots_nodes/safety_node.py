"""Safety supervisor node sitting between the worker and the controller (PLAN:irobots-v2:C26, I2).

Every commanded pose is checked by ``irobots_core.safety.SafetySupervisor``.
A rejected step is published as a first-class ``SkillResultMsg`` outcome and
is NEVER forwarded to ``ControlNode`` (T34).
"""

from __future__ import annotations

from collections.abc import Mapping

from irobots_core.robot_model import RobotModel
from irobots_core.safety import SafetySupervisor
from irobots_msgs.messages import SkillResultMsg

from .control_node import ControlNode


class SafetyNode:
    """Thin gatekeeper wrapping :class:`SafetySupervisor` in front of :class:`ControlNode` (C26)."""

    def __init__(self, robot: RobotModel, controller: ControlNode) -> None:
        self.supervisor = SafetySupervisor(robot)
        self.controller = controller

    @property
    def rejected_infeasible(self) -> int:
        return self.supervisor.rejected_infeasible

    def validate_and_forward(
        self,
        *,
        skill: str,
        target_id: str,
        goal_pose: Mapping[str, float],
        opening: float,
        deadline_s: float | None = None,
    ) -> SkillResultMsg:
        verdict = self.supervisor.check(goal_pose, opening=opening)
        if not verdict.approved:
            return SkillResultMsg(
                skill=skill,
                target_id=target_id,
                approved=False,
                outcome=f"safety_rejected:{verdict.reason.value}",
                elapsed_sim_s=0.0,
                timed_out=False,
            )
        return self.controller.execute_joint_goal(
            skill=skill,
            target_id=target_id,
            goal_pose=goal_pose,
            opening=opening,
            deadline_s=deadline_s,
        )
