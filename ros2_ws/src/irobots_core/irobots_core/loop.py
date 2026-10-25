"""Closed-loop manipulation executor (PLAN:irobots-v2:C36, C65, C67-F3, C67-F4, T46, T47, I1-I6).

Executes:
``observe -> plan/worker -> skill -> safety -> execute -> verify -> memory``

Enforces:
- I1: no Cartesian coordinate emitted by the planner/worker.
- I2: every pose checked by the safety supervisor before execution.
- I3: strictly one skill per observation (`max_skills_per_observation == 1`).
- I4: per-skill and per-episode deadlines.
- I5: privileged and perceived task success computed separately and labelled;
  real camera frames (`esp32_cam`) are never graded with privileged sim ground truth.
- I6: sim paused during perception and decision-making; stepped only by fixed dt.
- C67-F3: `esp32_cam` requires a validated `MarkerCalibration`, enforces the
  `allow_real_camera_with_sim_arm` hybrid guard when `arm_backend == "mujoco"`,
  labels hybrid runs `"hybrid: real camera + simulated arm"`, turns camera
  capture errors into a first-class `camera_unavailable` outcome, and logs
  `camera_source` + `capture_host` (without credentials) in every turn record.
- C67-F4: `EpisodeSession` makes the closed loop steppable one skill at a time
  so both `ClosedLoopRunner.run_episode` and the web console (`/api/step`,
  `/api/run`) share a single implementation.
"""

from __future__ import annotations

import json
import time
from collections.abc import Iterator, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .agent.memory import EpisodeMemory
from .agent.planner import TaskPlanner
from .agent.worker import RepeatActionEscalation, WorkerAgent
from .config import ConfigError, load_yaml
from .estop import EStopController
from .kinematics import inverse_kinematics
from .perception.camera_source import (
    HYBRID_MODE_LABEL,
    CameraCaptureError,
    CameraFrameSource,
    CameraSourceConfig,
    CameraSourceMode,
    build_camera_source,
)
from .perception.detector import filter_detections_by_vocabulary
from .perception.geometric import GeometricDetector
from .perception.grasp import propose_grasp_candidate
from .perception.height import HeightEstimator
from .perception.homography import AnalyticCalibration, Calibration, CameraModel, MarkerCalibration
from .perception.outcomes import SceneObservation, evaluate_outcome
from .perception.tracker import ObjectTracker, TrackedObject, TrackerConfig
from .replay import compute_action_log_sha256
from .robot_model import RobotModel
from .safety import SafetyReason, SafetySupervisor, Verdict
from .sim.mujoco_sim import create_sim
from .sim.scene_builder import SeededScene, TaskSpec, build_seeded_scene, load_task_spec
from .skills import SkillCall


@dataclass(frozen=True, slots=True)
class EpisodeSummary:
    episode_id: str
    seed: int
    task_name: str
    task_success_privileged: bool | None
    task_success_perceived: bool
    skills_executed: int
    observations_taken: int
    max_skills_per_observation: int
    safety_rejections: int
    sim_seconds: float
    action_log_sha256: str
    failure_mode: str = "none"
    camera_source: str = "sim"
    execution_mode: str = "sim"


class EpisodeSession:
    """Steppable single-episode state machine shared by batch eval and web console (C36, C67-F3, C67-F4)."""

    def __init__(
        self,
        runner: ClosedLoopRunner,
        *,
        task_path: str | Path = "config/tasks/l1.yaml",
        seed: int = 0,
        camera_source: CameraFrameSource | None = None,
        auto_approve_plan: bool = True,
    ) -> None:
        self.runner = runner
        self.seed = int(seed)
        self.task: TaskSpec = load_task_spec(task_path)
        self.seeded: SeededScene = build_seeded_scene(
            runner.robot, runner.scene_cfg, self.task, seed=self.seed
        )
        self.sim = create_sim(runner.robot, runner.scene_cfg, self.seeded.to_sim_objects())
        self.sim.reset(dict(runner.robot.home_pose), runner.robot.gripper.max_opening)
        self.sim.step(50)

        self.cam_src: CameraFrameSource = camera_source or build_camera_source(
            runner.camera_source_config, sim=self.sim
        )
        self.tracker = ObjectTracker(TrackerConfig(confirm_frames=1))
        self.supervisor = SafetySupervisor(runner.robot)
        self.planner = TaskPlanner()
        self.worker = WorkerAgent()
        self.episode_id = f"{self.task.name}_seed{self.seed}"
        self.memory = EpisodeMemory(episode_id=self.episode_id)

        self.auto_approve_plan = auto_approve_plan
        self.plan_approved = auto_approve_plan
        self.proposed_plan: list[SkillCall] = []
        self.remaining_plan: list[SkillCall] = []

        self.turn_index = 0
        self.observations_taken = 0
        self.skills_executed = 0
        self.max_skills_per_obs = 0
        self.turn_records: list[dict[str, Any]] = []
        self.target_poses_cache: dict[str, tuple[float, float, float, float]] = {}
        self.latest_tracks: list[TrackedObject] = []
        self.latest_track_details: list[dict[str, Any]] = []
        self.latest_frame_rgb: np.ndarray | None = None
        self.last_verdict: Verdict = Verdict(True, SafetyReason.OK, "ok")
        self.last_predicates: dict[str, bool] = {
            "gripper_at_pose": True,
            "object_moved_with_gripper": False,
            "object_lifted": False,
            "object_knocked_away": False,
            "no_target": False,
            "table_hit": False,
        }
        self.failure_mode = "none"
        self.done = False
        self._closed = False
        self._wall_start = time.monotonic()

    @property
    def execution_mode(self) -> str:
        if self.cam_src.mode is CameraSourceMode.ESP32_CAM and self.runner.arm_backend == "mujoco":
            return HYBRID_MODE_LABEL
        return self.cam_src.mode.value

    def _turn_metadata(self) -> dict[str, Any]:
        meta: dict[str, Any] = {
            "camera_source": self.cam_src.mode.value,
        }
        if self.cam_src.mode is CameraSourceMode.ESP32_CAM:
            meta["capture_host"] = self.cam_src.capture_host
            meta["execution_mode"] = self.execution_mode
        return meta

    def observe_scene(self) -> bool:
        """Capture one frame while the simulator is paused (I6) and update tracks."""
        remaining_wall = max(0.05, self.task.episode_deadline_s - (time.monotonic() - self._wall_start))
        captured = self.cam_src.capture(
            sim_time_s=self.sim.total_steps * self.sim.dt,
            deadline_s=min(5.0, remaining_wall),
        )
        if isinstance(captured, CameraCaptureError):
            rec: dict[str, Any] = {
                "turn_index": self.turn_index,
                "sim_time_s": round(self.sim.total_steps * self.sim.dt, 6),
                "skill": "observe",
                "target_id": "",
                "safety_approved": False,
                "outcome": "camera_unavailable",
                **self._turn_metadata(),
            }
            self.turn_records.append(rec)
            self.failure_mode = "camera_unavailable"
            self.done = True
            return False

        self.latest_frame_rgb = captured.rgb
        self.observations_taken += 1

        raw_dets = self.runner.detector.detect(captured.rgb)
        dets = filter_detections_by_vocabulary(raw_dets, self.runner.vocabulary)
        xy_list: list[tuple[float, float]] = []
        raw_xy_list: list[tuple[float, float]] = []
        h_list: list[tuple[float, str]] = []
        bottom_px_list: list[tuple[float, float]] = []

        for d in dets:
            h_est = self.runner.height_estimator.estimate(d.label)
            cand = propose_grasp_candidate(
                detection=d,
                calibration=self.runner.calibration,
                camera_xy=(self.runner.camera.position[0], self.runner.camera.position[1]),
                height=h_est,
                max_gripper_opening_m=self.runner.robot.gripper.max_opening,
            )
            xy_list.append(cand.contact_xy_m)
            raw_xy_list.append(cand.raw_xy_m)
            h_list.append((h_est.value, h_est.source.value))
            bottom_px_list.append(d.bottom_center_px)

        tracks = self.tracker.update(dets, xy_list)
        self.latest_tracks = tracks
        details: list[dict[str, Any]] = []
        for idx, tr in enumerate(tracks):
            self.target_poses_cache[tr.track_id] = (tr.xy_m[0], tr.xy_m[1], 0.020, 0.060)
            raw_xy = raw_xy_list[idx] if idx < len(raw_xy_list) else tr.xy_m
            h_val, h_src = h_list[idx] if idx < len(h_list) else (0.040, "prior")
            b_px = bottom_px_list[idx] if idx < len(bottom_px_list) else (320.0, 240.0)
            details.append(
                {
                    "track_id": tr.track_id,
                    "label": tr.label,
                    "status": tr.status.value,
                    "bottom_px": [round(float(b_px[0]), 2), round(float(b_px[1]), 2)],
                    "raw_xy_m": [round(float(raw_xy[0]), 4), round(float(raw_xy[1]), 4)],
                    "xy_m": [round(float(tr.xy_m[0]), 4), round(float(tr.xy_m[1]), 4)],
                    "height_m": round(float(h_val), 4),
                    "height_source": h_src,
                }
            )
        self.latest_track_details = details
        return True

    def propose_plan_from_text(self, instruction_text: str) -> list[SkillCall]:
        """Generate and validate a coordinate-free skill plan against tracked scene objects (T40)."""
        if not self.target_poses_cache:
            self.observe_scene()
        active_ids = set(self.target_poses_cache.keys())
        text_low = instruction_text.lower()

        # Check if prompt names a specific colour/object that is NOT in the scene (T40)
        for colour in ("red", "blue", "green", "yellow", "purple", "orange"):
            if colour in text_low and not any(colour in tid for tid in active_ids):
                raise ValueError(
                    f"PlanApprovalError: requested '{colour}' object is not present in tracked scene IDs {sorted(active_ids)}"
                )

        if "blue" in text_low:
            target_id = next((tid for tid in active_ids if "blue" in tid), "")
        elif "red" in text_low:
            target_id = next((tid for tid in active_ids if "red" in tid), "")
        else:
            target_id = next(
                (tid for tid in active_ids if "red" in tid),
                next(iter(active_ids), ""),
            )

        if not target_id:
            raise ValueError("PlanApprovalError: no matching target object in scene")

        wants_bin = any(w in text_low for w in ("bin", "sort", "place"))
        bin_id = next((tid for tid in active_ids if "bin" in tid or "green" in tid), None)
        if wants_bin and bin_id is None:
            raise ValueError("PlanApprovalError: task requires a bin, but no bin is tracked in scene")

        goal_type = "place_in_bin" if (wants_bin and bin_id) else "lift"
        proposed = self.planner.plan_for_task(goal_type, target_id, bin_id=bin_id)
        approved = self.planner.validate_and_approve(proposed, active_ids)
        self.proposed_plan = list(approved)
        self.remaining_plan = list(approved)
        self.plan_approved = False
        return list(approved)

    def approve_plan(self) -> None:
        if not self.remaining_plan and self.proposed_plan:
            self.remaining_plan = list(self.proposed_plan)
        self.plan_approved = True

    def step_turn(self) -> dict[str, Any]:
        """Execute strictly ONE observation and ONE skill turn (Invariants I1, I2, I3, I4, I6)."""
        if self.done:
            return {"done": True, "failure_mode": self.failure_mode}
        if self.runner.estop.triggered:
            self.failure_mode = "estop_aborted"
            self.done = True
            return {"done": True, "failure_mode": self.failure_mode}
        if (
            self.sim.total_steps * self.sim.dt >= self.task.episode_deadline_s
            or (time.monotonic() - self._wall_start) >= self.task.episode_deadline_s
        ):
            self.failure_mode = "timeout"
            self.done = True
            return {"done": True, "failure_mode": self.failure_mode}
        if self.turn_index >= self.task.turn_budget:
            self.done = True
            return {"done": True, "failure_mode": self.failure_mode}

        # 1. OBSERVE (sim paused: Invariant I6)
        if not self.observe_scene():
            return {"done": True, "failure_mode": self.failure_mode}

        skills_this_obs = 0
        active_ids = set(self.target_poses_cache.keys())
        if not active_ids:
            self.failure_mode = "no_target"
            self.done = True
            return {"done": True, "failure_mode": self.failure_mode}

        # 2. PLAN (if not yet proposed)
        if not self.proposed_plan and not self.remaining_plan:
            target_id = next(
                (tid for tid in active_ids if "red" in tid),
                next(iter(active_ids)),
            )
            bin_id = next((tid for tid in active_ids if "bin" in tid or "green" in tid), None)
            proposed = self.planner.plan_for_task(self.task.goal_type, target_id, bin_id=bin_id)
            approved = self.planner.validate_and_approve(proposed, active_ids)
            self.proposed_plan = list(approved)
            self.remaining_plan = list(approved)
            if self.auto_approve_plan:
                self.plan_approved = True

        if not self.plan_approved:
            raise PermissionError("Plan must be approved before executing skills (D11)")

        if not self.remaining_plan:
            self.done = True
            return {"done": True, "failure_mode": self.failure_mode}

        # 3. WORKER (strictly ONE skill per observation: Invariant I3)
        try:
            call = self.worker.select_next_skill(self.remaining_plan, active_ids, self.memory)
        except RepeatActionEscalation:
            self.failure_mode = "planner_loop"
            self.done = True
            return {"done": True, "failure_mode": self.failure_mode}

        skills_this_obs += 1
        self.max_skills_per_obs = max(self.max_skills_per_obs, skills_this_obs)

        # 4. RESOLVE TARGET POSE & SAFETY SUPERVISOR (Invariants I1, I2)
        tid_key = call.target_id or next(iter(self.target_poses_cache.keys()))
        tx, ty, grasp_z, hover_z = self.target_poses_cache[tid_key]
        if call.skill in ("move_above", "retreat"):
            desired_xyz = (tx, ty, hover_z)
            opening = (
                self.runner.robot.gripper.max_opening
                if call.skill == "move_above" and self.skills_executed == 0
                else self.sim.opening()
            )
        elif call.skill == "grasp":
            desired_xyz = (tx, ty, grasp_z)
            opening = self.runner.robot.gripper.min_opening
        elif call.skill == "lift":
            desired_xyz = (tx, ty, hover_z)
            opening = self.runner.robot.gripper.min_opening
        elif call.skill == "release":
            tip_now = self.sim.tool_position()
            desired_xyz = (float(tip_now[0]), float(tip_now[1]), float(tip_now[2]))
            opening = self.runner.robot.gripper.max_opening
        else:  # home
            desired_xyz = (0.0, 0.20, 0.12)
            opening = self.runner.robot.gripper.max_opening

        ik = inverse_kinematics(self.runner.robot, desired_xyz)
        if not ik.ok:
            self.failure_mode = "ik_unreachable"
            self.memory.record_step(call, outcome="ik_unreachable", success=False)
            self.done = True
            return {"done": True, "failure_mode": self.failure_mode}

        verdict = self.supervisor.check(ik.pose, opening=opening)
        self.last_verdict = verdict
        if not verdict.approved:
            outcome_str = f"safety_rejected:{verdict.reason.value}"
            self.memory.record_step(call, outcome=outcome_str, success=False)
            rec = {
                "turn_index": self.turn_index,
                "sim_time_s": round(self.sim.total_steps * self.sim.dt, 6),
                "skill": call.skill,
                "target_id": call.target_id or "",
                "safety_approved": False,
                "outcome": outcome_str,
                **self._turn_metadata(),
            }
            self.turn_records.append(rec)
            self.turn_index += 1
            self.failure_mode = "safety_rejected"
            self.done = True
            return rec

        # 5. EXECUTE IN FIXED-STEP SIM (checking E-Stop within one control cycle: T52)
        tip_before = self.sim.tool_position()
        obj_before = self.sim.object_position(self.task.target_object)
        before_obs = SceneObservation(
            gripper_xyz=(float(tip_before[0]), float(tip_before[1]), float(tip_before[2])),
            gripper_opening=self.sim.opening(),
            objects={
                self.task.target_object: (
                    float(obj_before[0]),
                    float(obj_before[1]),
                    float(obj_before[2]),
                )
            },
        )

        if call.skill == "grasp":
            self.sim.set_opening(self.runner.robot.gripper.max_opening)
            _, aborted1 = self.runner.estop.step_guarded(self.sim, goal_pose=ik.pose, max_steps=400)
            if not aborted1:
                self.sim.set_opening(self.runner.robot.gripper.min_opening)
                _, aborted2 = self.runner.estop.step_guarded(self.sim, goal_pose=ik.pose, max_steps=400)
            else:
                aborted2 = True
            aborted = aborted1 or aborted2
        else:
            self.sim.set_opening(opening)
            _, aborted = self.runner.estop.step_guarded(self.sim, goal_pose=ik.pose, max_steps=400)

        if aborted or self.runner.estop.triggered:
            self.failure_mode = "estop_aborted"
            self.done = True
            rec_abort = {
                "turn_index": self.turn_index,
                "sim_time_s": round(self.sim.total_steps * self.sim.dt, 6),
                "skill": call.skill,
                "target_id": call.target_id or "",
                "safety_approved": True,
                "outcome": "estop_aborted",
                **self._turn_metadata(),
            }
            self.turn_records.append(rec_abort)
            self.turn_index += 1
            return rec_abort

        self.skills_executed += 1

        # 6. VERIFY OUTCOME & UPDATE MEMORY
        tip_after = self.sim.tool_position()
        obj_after = self.sim.object_position(self.task.target_object)
        after_obs = SceneObservation(
            gripper_xyz=(float(tip_after[0]), float(tip_after[1]), float(tip_after[2])),
            gripper_opening=self.sim.opening(),
            objects={
                self.task.target_object: (
                    float(obj_after[0]),
                    float(obj_after[1]),
                    float(obj_after[2]),
                )
            },
        )
        ev = evaluate_outcome(
            before_obs, after_obs, self.task.target_object, desired_xyz, self.runner.scene_cfg
        )
        self.last_predicates = dict(ev.predicates)
        outcome_str = ev.primary.value
        self.memory.record_step(call, outcome=outcome_str, success=True)
        rec = {
            "turn_index": self.turn_index,
            "sim_time_s": round(self.sim.total_steps * self.sim.dt, 6),
            "skill": call.skill,
            "target_id": call.target_id or "",
            "safety_approved": True,
            "outcome": outcome_str,
            **self._turn_metadata(),
        }
        self.turn_records.append(rec)
        self.turn_index += 1
        self.remaining_plan.pop(0)
        if not self.remaining_plan or self.turn_index >= self.task.turn_budget:
            self.done = True
        return rec

    def __iter__(self) -> Iterator[dict[str, Any]]:
        while not self.done:
            yield self.step_turn()

    def current_summary(self) -> EpisodeSummary:
        """Compute current EpisodeSummary without closing the simulator."""
        perc_success = any(r["outcome"] == "object_lifted" for r in self.turn_records)
        if self.cam_src.mode is CameraSourceMode.ESP32_CAM:
            # F3 / I5: Real frames must never be graded with privileged sim ground truth.
            priv_success: bool | None = None
        else:
            obj_final = self.sim.object_position(self.task.target_object)
            tip_final = self.sim.tool_position()
            bottom_clearance = float(obj_final[2] - 0.020)
            slip_m = float(
                ((obj_final[0] - tip_final[0]) ** 2 + (obj_final[1] - tip_final[1]) ** 2) ** 0.5
            )
            priv_success = bool(bottom_clearance > 0.015 and slip_m < 0.035)

        sim_sec = self.sim.total_steps * self.sim.dt
        log_hash = compute_action_log_sha256(self.turn_records)
        return EpisodeSummary(
            episode_id=self.episode_id,
            seed=self.seed,
            task_name=self.task.name,
            task_success_privileged=priv_success,
            task_success_perceived=perc_success,
            skills_executed=self.skills_executed,
            observations_taken=self.observations_taken,
            max_skills_per_observation=self.max_skills_per_obs,
            safety_rejections=self.supervisor.rejected_infeasible,
            sim_seconds=sim_sec,
            action_log_sha256=log_hash,
            failure_mode=self.failure_mode,
            camera_source=self.cam_src.mode.value,
            execution_mode=self.execution_mode,
        )

    def finish(
        self,
        *,
        jsonl_path: str | Path | None = None,
        metrics_path: str | Path | None = None,
    ) -> EpisodeSummary:
        summary = self.current_summary()
        if not self._closed:
            self.sim.close()
            self._closed = True

        if jsonl_path is not None:
            jp = Path(jsonl_path)
            jp.parent.mkdir(parents=True, exist_ok=True)
            lines = [json.dumps(r, sort_keys=True) for r in self.turn_records]
            lines.append(
                json.dumps(
                    {
                        "type": "episode_summary",
                        "episode_id": summary.episode_id,
                        "action_log_sha256": summary.action_log_sha256,
                        "task_success_privileged": summary.task_success_privileged,
                        "task_success_perceived": summary.task_success_perceived,
                        "camera_source": summary.camera_source,
                        "execution_mode": summary.execution_mode,
                        "failure_mode": summary.failure_mode,
                    },
                    sort_keys=True,
                )
            )
            jp.write_text("\n".join(lines) + "\n", encoding="utf-8")

        if metrics_path is not None:
            mp = Path(metrics_path)
            mp.parent.mkdir(parents=True, exist_ok=True)
            mp.write_text(
                json.dumps(
                    {
                        "episode_id": summary.episode_id,
                        "seed": summary.seed,
                        "task_name": summary.task_name,
                        "camera_source": summary.camera_source,
                        "execution_mode": summary.execution_mode,
                        "task_success_privileged": summary.task_success_privileged,
                        "task_success_perceived": summary.task_success_perceived,
                        "skills_executed": summary.skills_executed,
                        "observations_taken": summary.observations_taken,
                        "safety_rejections": summary.safety_rejections,
                        "sim_seconds": summary.sim_seconds,
                        "action_log_sha256": summary.action_log_sha256,
                        "failure_mode": summary.failure_mode,
                    },
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )

        return summary


class ClosedLoopRunner:
    """Closed-loop perception -> planner -> worker -> safety -> sim -> verification runner."""

    def __init__(
        self,
        robot: RobotModel,
        scene_cfg: Mapping[str, Any],
        *,
        estop: EStopController | None = None,
        camera_source_override: str | CameraSourceMode | None = None,
        calibration_file: str | Path | None = None,
        allow_real_camera_with_sim_arm: bool | None = None,
        marker_calibration: MarkerCalibration | None = None,
        arm_backend: str = "mujoco",
    ) -> None:
        self.robot = robot
        self.scene_cfg = scene_cfg
        self.estop = estop or EStopController()
        self.arm_backend = arm_backend.strip().lower()
        p_cfg = load_yaml("config/perception.yaml")["perception"]
        cam_cfg = scene_cfg["camera"]
        pos_raw = cam_cfg["position"]
        tgt_raw = cam_cfg["target"]
        self.camera = CameraModel(
            position=(float(pos_raw[0]), float(pos_raw[1]), float(pos_raw[2])),
            target=(float(tgt_raw[0]), float(tgt_raw[1]), float(tgt_raw[2])),
            fov_deg=float(cam_cfg["hfov_deg"]),
            width=int(cam_cfg["width"]),
            height=int(cam_cfg["height"]),
        )
        self.camera_source_config = CameraSourceConfig.from_configs(
            p_cfg,
            scene_cfg,
            source_override=camera_source_override,
            calibration_file_override=calibration_file,
            allow_real_camera_with_sim_arm_override=allow_real_camera_with_sim_arm,
            marker_calibration_override=marker_calibration,
        )

        # F3 Hybrid guard: esp32_cam + mujoco arm backend requires explicit allow_real_camera_with_sim_arm
        if (
            self.camera_source_config.source is CameraSourceMode.ESP32_CAM
            and self.arm_backend == "mujoco"
            and not self.camera_source_config.allow_real_camera_with_sim_arm
        ):
            raise ConfigError(
                "Hybrid guard: camera_source is 'esp32_cam' while arm_backend is 'mujoco'; "
                "set perception.camera_source.allow_real_camera_with_sim_arm=True to authorize hybrid execution"
            )

        if self.camera_source_config.source is CameraSourceMode.ESP32_CAM:
            if self.camera_source_config.marker_calibration is None:
                raise ConfigError("esp32_cam requires a validated MarkerCalibration")
            self.calibration: Calibration = self.camera_source_config.marker_calibration
        else:
            self.calibration = AnalyticCalibration(camera=self.camera)

        self.detector = GeometricDetector(
            camera=self.camera,
            workspace_centre=robot.safety.workspace_centre,
            workspace_radius=robot.safety.workspace_radius,
        )
        self.height_estimator = HeightEstimator.from_config(p_cfg, camera=self.camera)
        self.vocabulary: list[str] = list(
            p_cfg.get("vocabulary", ["red block", "blue block", "bin"])
        )

    def start_episode(
        self,
        *,
        task_path: str | Path = "config/tasks/l1.yaml",
        seed: int = 0,
        camera_source: CameraFrameSource | None = None,
        auto_approve_plan: bool = True,
    ) -> EpisodeSession:
        return EpisodeSession(
            self,
            task_path=task_path,
            seed=seed,
            camera_source=camera_source,
            auto_approve_plan=auto_approve_plan,
        )

    def run_episode(
        self,
        *,
        task_path: str | Path = "config/tasks/l1.yaml",
        seed: int = 0,
        jsonl_path: str | Path | None = None,
        metrics_path: str | Path | None = None,
        camera_source: CameraFrameSource | None = None,
    ) -> EpisodeSummary:
        session = self.start_episode(
            task_path=task_path,
            seed=seed,
            camera_source=camera_source,
            auto_approve_plan=True,
        )
        for _ in session:
            pass
        return session.finish(jsonl_path=jsonl_path, metrics_path=metrics_path)
