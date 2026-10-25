"""Closed-loop manipulation executor (PLAN:irobots-v2:C36, T46, T47, I1-I6).

Executes:
``observe -> plan/worker -> skill -> safety -> execute -> verify -> memory``

Enforces:
- I1: no Cartesian coordinate emitted by the planner/worker.
- I2: every pose checked by the safety supervisor before execution.
- I3: strictly one skill per observation (`max_skills_per_observation == 1`).
- I4: per-skill and per-episode deadlines.
- I5: privileged and perceived task success computed separately and labelled.
- I6: sim paused during perception and decision-making; stepped only by fixed dt.
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .agent.memory import EpisodeMemory
from .agent.planner import TaskPlanner
from .agent.worker import RepeatActionEscalation, WorkerAgent
from .config import load_yaml
from .estop import EStopController
from .kinematics import inverse_kinematics
from .perception.detector import filter_detections_by_vocabulary
from .perception.geometric import GeometricDetector
from .perception.grasp import propose_grasp_candidate
from .perception.height import HeightEstimator
from .perception.homography import AnalyticCalibration, CameraModel
from .perception.outcomes import SceneObservation, evaluate_outcome
from .perception.tracker import ObjectTracker, TrackerConfig
from .replay import compute_action_log_sha256
from .robot_model import RobotModel
from .safety import SafetySupervisor
from .sim.mujoco_sim import create_sim
from .sim.scene_builder import build_seeded_scene, load_task_spec


@dataclass(frozen=True, slots=True)
class EpisodeSummary:
    episode_id: str
    seed: int
    task_name: str
    task_success_privileged: bool
    task_success_perceived: bool
    skills_executed: int
    observations_taken: int
    max_skills_per_observation: int
    safety_rejections: int
    sim_seconds: float
    action_log_sha256: str
    failure_mode: str = "none"


class ClosedLoopRunner:
    """Closed-loop perception -> planner -> worker -> safety -> sim -> verification runner."""

    def __init__(
        self,
        robot: RobotModel,
        scene_cfg: Mapping[str, Any],
        *,
        estop: EStopController | None = None,
    ) -> None:
        self.robot = robot
        self.scene_cfg = scene_cfg
        self.estop = estop or EStopController()
        p_cfg = load_yaml("config/perception.yaml")["perception"]
        cam_cfg = scene_cfg["camera"]
        self.camera = CameraModel(
            position=tuple(float(c) for c in cam_cfg["position"]),  # type: ignore[arg-type]
            target=tuple(float(c) for c in cam_cfg["target"]),  # type: ignore[arg-type]
            fov_deg=float(cam_cfg["hfov_deg"]),
            width=int(cam_cfg["width"]),
            height=int(cam_cfg["height"]),
        )
        self.calibration = AnalyticCalibration(camera=self.camera)
        self.detector = GeometricDetector(
            camera=self.camera,
            workspace_centre=robot.safety.workspace_centre,
            workspace_radius=robot.safety.workspace_radius,
        )
        self.height_estimator = HeightEstimator.from_config(p_cfg, camera=self.camera)
        self.vocabulary: list[str] = list(p_cfg.get("vocabulary", ["red block", "blue block", "bin"]))

    def run_episode(
        self,
        *,
        task_path: str | Path = "config/tasks/l1.yaml",
        seed: int = 0,
        jsonl_path: str | Path | None = None,
    ) -> EpisodeSummary:
        task = load_task_spec(task_path)
        seeded = build_seeded_scene(self.robot, self.scene_cfg, task, seed=seed)
        sim = create_sim(self.robot, self.scene_cfg, seeded.to_sim_objects())
        sim.reset(dict(self.robot.home_pose), self.robot.gripper.max_opening)
        sim.step(50)

        tracker = ObjectTracker(TrackerConfig(confirm_frames=1))
        supervisor = SafetySupervisor(self.robot)
        planner = TaskPlanner()
        worker = WorkerAgent()
        ep_id = f"{task.name}_seed{seed}"
        memory = EpisodeMemory(episode_id=ep_id)

        observations_taken = 0
        skills_executed = 0
        max_skills_per_obs = 0
        turn_records: list[dict[str, Any]] = []
        target_poses_cache: dict[str, tuple[float, float, float, float]] = {}
        plan = None
        failure_mode = "none"

        for turn in range(task.turn_budget):
            if self.estop.triggered:
                failure_mode = "estop_aborted"
                break
            if sim.total_steps * sim.dt >= task.episode_deadline_s:
                failure_mode = "timeout"
                break

            # 1. OBSERVE (sim is paused during observation and planning: Invariant I6)
            frame = sim.render()
            observations_taken += 1
            skills_this_obs = 0

            raw_dets = self.detector.detect(frame)
            dets = filter_detections_by_vocabulary(raw_dets, self.vocabulary)
            xy_list: list[tuple[float, float]] = []
            for d in dets:
                h_est = self.height_estimator.estimate(d.label)
                cand = propose_grasp_candidate(
                    detection=d,
                    calibration=self.calibration,
                    camera_xy=(self.camera.position[0], self.camera.position[1]),
                    height=h_est,
                    max_gripper_opening_m=self.robot.gripper.max_opening,
                )
                xy_list.append(cand.contact_xy_m)
            tracks = tracker.update(dets, xy_list)
            for tr in tracks:
                target_poses_cache[tr.track_id] = (tr.xy_m[0], tr.xy_m[1], 0.020, 0.060)

            active_ids = set(target_poses_cache.keys())
            if not active_ids:
                failure_mode = "no_target"
                break

            # 2. PLAN (on first turn or replan)
            if plan is None:
                target_id = next(
                    (tid for tid in active_ids if "red" in tid),
                    next(iter(active_ids)),
                )
                bin_id = next((tid for tid in active_ids if "bin" in tid or "green" in tid), None)
                proposed = planner.plan_for_task(task.goal_type, target_id, bin_id=bin_id)
                plan = planner.validate_and_approve(proposed, active_ids)

            if not plan:
                break

            # 3. WORKER (selects strictly ONE skill per observation: Invariant I3)
            try:
                call = worker.select_next_skill(plan, active_ids, memory)
            except RepeatActionEscalation:
                failure_mode = "planner_loop"
                break

            skills_this_obs += 1
            max_skills_per_obs = max(max_skills_per_obs, skills_this_obs)

            # 4. RESOLVE TARGET POSE & SAFETY SUPERVISOR (Invariant I1, I2)
            tid_key = call.target_id or next(iter(target_poses_cache.keys()))
            tx, ty, grasp_z, hover_z = target_poses_cache[tid_key]
            if call.skill in ("move_above", "retreat"):
                desired_xyz = (tx, ty, hover_z)
                opening = (
                    self.robot.gripper.max_opening
                    if call.skill == "move_above" and skills_executed == 0
                    else sim.opening()
                )
            elif call.skill == "grasp":
                desired_xyz = (tx, ty, grasp_z)
                opening = self.robot.gripper.min_opening
            elif call.skill == "lift":
                desired_xyz = (tx, ty, hover_z)
                opening = self.robot.gripper.min_opening
            elif call.skill == "release":
                tip_now = sim.tool_position()
                desired_xyz = (float(tip_now[0]), float(tip_now[1]), float(tip_now[2]))
                opening = self.robot.gripper.max_opening
            else:  # home
                desired_xyz = (0.0, 0.20, 0.12)
                opening = self.robot.gripper.max_opening

            ik = inverse_kinematics(self.robot, desired_xyz)
            if not ik.ok:
                failure_mode = "ik_unreachable"
                memory.record_step(call, outcome="ik_unreachable", success=False)
                break

            verdict = supervisor.check(ik.pose, opening=opening)
            if not verdict.approved:
                outcome_str = f"safety_rejected:{verdict.reason.value}"
                memory.record_step(call, outcome=outcome_str, success=False)
                turn_records.append(
                    {
                        "turn_index": turn,
                        "sim_time_s": round(sim.total_steps * sim.dt, 6),
                        "skill": call.skill,
                        "target_id": call.target_id or "",
                        "safety_approved": False,
                        "outcome": outcome_str,
                    }
                )
                failure_mode = "safety_rejected"
                break

            # 5. EXECUTE IN FIXED-STEP SIM
            tip_before = sim.tool_position()
            obj_before = sim.object_position(task.target_object)
            before_obs = SceneObservation(
                gripper_xyz=(float(tip_before[0]), float(tip_before[1]), float(tip_before[2])),
                gripper_opening=sim.opening(),
                objects={task.target_object: (float(obj_before[0]), float(obj_before[1]), float(obj_before[2]))},
            )

            if call.skill == "grasp":
                sim.set_joint_targets(ik.pose)
                sim.set_opening(self.robot.gripper.max_opening)
                sim.step_until_settled(max_steps=400)
                sim.set_opening(self.robot.gripper.min_opening)
                sim.step_until_settled(max_steps=400)
            else:
                sim.set_joint_targets(ik.pose)
                sim.set_opening(opening)
                sim.step_until_settled(max_steps=400)

            skills_executed += 1

            # 6. VERIFY OUTCOME & UPDATE MEMORY
            tip_after = sim.tool_position()
            obj_after = sim.object_position(task.target_object)
            after_obs = SceneObservation(
                gripper_xyz=(float(tip_after[0]), float(tip_after[1]), float(tip_after[2])),
                gripper_opening=sim.opening(),
                objects={task.target_object: (float(obj_after[0]), float(obj_after[1]), float(obj_after[2]))},
            )
            ev = evaluate_outcome(before_obs, after_obs, task.target_object, desired_xyz, self.scene_cfg)
            outcome_str = ev.primary.value
            memory.record_step(call, outcome=outcome_str, success=True)
            turn_records.append(
                {
                    "turn_index": turn,
                    "sim_time_s": round(sim.total_steps * sim.dt, 6),
                    "skill": call.skill,
                    "target_id": call.target_id or "",
                    "safety_approved": True,
                    "outcome": outcome_str,
                }
            )
            plan.pop(0)
            if not plan:
                break

        # Final grading (Invariant I5: privileged and perceived separated)
        obj_final = sim.object_position(task.target_object)
        tip_final = sim.tool_position()
        bottom_clearance = float(obj_final[2] - 0.020)
        slip_m = float(((obj_final[0] - tip_final[0]) ** 2 + (obj_final[1] - tip_final[1]) ** 2) ** 0.5)
        priv_success = bool(bottom_clearance > 0.015 and slip_m < 0.035)
        perc_success = any(r["outcome"] == "object_lifted" for r in turn_records)
        sim_sec = sim.total_steps * sim.dt
        sim.close()

        log_hash = compute_action_log_sha256(turn_records)
        if jsonl_path is not None:
            jp = Path(jsonl_path)
            jp.parent.mkdir(parents=True, exist_ok=True)
            lines = [json.dumps(r, sort_keys=True) for r in turn_records]
            lines.append(
                json.dumps(
                    {
                        "type": "episode_summary",
                        "episode_id": ep_id,
                        "action_log_sha256": log_hash,
                        "task_success_privileged": priv_success,
                        "task_success_perceived": perc_success,
                    },
                    sort_keys=True,
                )
            )
            jp.write_text("\n".join(lines) + "\n", encoding="utf-8")

        return EpisodeSummary(
            episode_id=ep_id,
            seed=seed,
            task_name=task.name,
            task_success_privileged=priv_success,
            task_success_perceived=perc_success,
            skills_executed=skills_executed,
            observations_taken=observations_taken,
            max_skills_per_observation=max_skills_per_obs,
            safety_rejections=supervisor.rejected_infeasible,
            sim_seconds=sim_sec,
            action_log_sha256=log_hash,
            failure_mode=failure_mode,
        )
