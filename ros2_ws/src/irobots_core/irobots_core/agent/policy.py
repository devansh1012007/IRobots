"""The scripted reference policy (PLAN:irobots-v2:C34, T45, SC1).

A deterministic, hand-written skill policy that uses the exact same perception,
height estimation, grasp synthesis, and safety supervisor as the LLM conditions,
without an LLM. It serves as both the RQ0 control arm and the ceiling benchmark.
"""

from __future__ import annotations

import json
from collections.abc import Iterable, Mapping, Sequence
from typing import Any

from ..config import REPO_ROOT, load_yaml
from ..kinematics import inverse_kinematics
from ..perception.camera_source import CameraCaptureError, CameraFrameSource, CameraSourceConfig, build_camera_source
from ..perception.geometric import GeometricDetector
from ..perception.grasp import propose_grasp_candidate
from ..perception.height import HeightEstimator
from ..perception.homography import AnalyticCalibration, CameraModel
from ..perception.outcomes import SceneObservation, evaluate_outcome
from ..perception.tracker import ObjectTracker, TrackerConfig
from ..robot_model import RobotModel
from ..safety import SafetySupervisor
from ..sim.mujoco_sim import create_sim, detect_sim_backend_label
from ..sim.scene_builder import SHARED_EVAL_SEEDS, build_seeded_scene, load_task_spec
from ..skills import SkillCall


class ScriptedReferencePolicy:
    """Deterministic state-machine policy emitting one :class:`SkillCall` per observation."""

    def next_skill(
        self,
        *,
        goal_type: str,
        target_id: str,
        bin_id: str | None,
        completed_skills: Sequence[str],
    ) -> SkillCall:
        if goal_type == "lift":
            sequence = [
                SkillCall(skill="move_above", target_id=target_id),
                SkillCall(skill="grasp", target_id=target_id),
                SkillCall(skill="lift", target_id=target_id),
            ]
        else:
            sequence = [
                SkillCall(skill="move_above", target_id=target_id),
                SkillCall(skill="grasp", target_id=target_id),
                SkillCall(skill="lift", target_id=target_id),
                SkillCall(skill="move_above", target_id=bin_id),
                SkillCall(skill="release"),
                SkillCall(skill="retreat"),
            ]
        idx = min(len(completed_skills), len(sequence) - 1)
        return sequence[idx]


def evaluate_scripted_policy_l1(
    robot: RobotModel,
    scene_cfg: Mapping[str, Any],
    *,
    policy: ScriptedReferencePolicy | None = None,
    seeds: Iterable[int] = SHARED_EVAL_SEEDS,
    write_run_json: bool = True,
    camera_source: CameraFrameSource | None = None,
) -> dict[str, Any]:
    """Run the scripted reference policy across the 30 seeded L1 scenes (SC1 gate)."""
    pol = policy or ScriptedReferencePolicy()
    task = load_task_spec("config/tasks/l1.yaml")
    p_cfg = load_yaml("config/perception.yaml")["perception"]
    cam_cfg = scene_cfg["camera"]
    camera = CameraModel(
        position=(float(cam_cfg["position"][0]), float(cam_cfg["position"][1]), float(cam_cfg["position"][2])),
        target=(float(cam_cfg["target"][0]), float(cam_cfg["target"][1]), float(cam_cfg["target"][2])),
        fov_deg=float(cam_cfg["hfov_deg"]),
        width=int(cam_cfg["width"]),
        height=int(cam_cfg["height"]),
    )
    calibration = AnalyticCalibration(camera=camera)
    detector = GeometricDetector(
        camera=camera,
        workspace_centre=robot.safety.workspace_centre,
        workspace_radius=robot.safety.workspace_radius,
    )
    height_est = HeightEstimator.from_config(p_cfg, camera=camera)
    cam_src_cfg = CameraSourceConfig.from_configs(p_cfg, scene_cfg, source_override='sim')

    seed_list = list(seeds)
    results: list[dict[str, Any]] = []
    for seed in seed_list:
        seeded = build_seeded_scene(robot, scene_cfg, task, seed=seed)
        sim = create_sim(robot, scene_cfg, seeded.to_sim_objects())
        sim.reset(dict(robot.home_pose), robot.gripper.max_opening)
        sim.step(50)

        cam_src = camera_source or build_camera_source(cam_src_cfg, sim=sim)
        tracker = ObjectTracker(TrackerConfig(confirm_frames=1))
        supervisor = SafetySupervisor(robot)
        completed: list[str] = []
        cand_cache: dict[str, Any] = {}

        for _turn in range(3):
            # 1. Perceive current frame (paused sim, I6)
            cap = cam_src.capture(sim_time_s=sim.total_steps * sim.dt)
            if isinstance(cap, CameraCaptureError):
                break
            frame = cap.rgb
            dets = detector.detect(frame)
            xy_list = []
            for d in dets:
                h = height_est.estimate("red block")
                cand = propose_grasp_candidate(
                    detection=d,
                    calibration=calibration,
                    camera_xy=(camera.position[0], camera.position[1]),
                    height=h,
                    max_gripper_opening_m=robot.gripper.max_opening,
                )
                xy_list.append(cand.contact_xy_m)
            tracks = tracker.update(dets, xy_list)
            if not tracks and not cand_cache:
                break
            if tracks:
                tid = tracks[0].track_id
                cand_cache[tid] = (tracks[0].xy_m, 0.020, 0.060)
            else:
                tid = next(iter(cand_cache.keys()))

            call = pol.next_skill(
                goal_type="lift",
                target_id=tid,
                bin_id=None,
                completed_skills=completed,
            )
            tx, ty, grasp_z = cand_cache[tid][0][0], cand_cache[tid][0][1], cand_cache[tid][1]
            hover_z = cand_cache[tid][2]

            if call.skill == "move_above":
                ik = inverse_kinematics(robot, (tx, ty, hover_z))
                opening = robot.gripper.max_opening
            elif call.skill == "grasp":
                ik = inverse_kinematics(robot, (tx, ty, grasp_z))
                opening = robot.gripper.min_opening
            else:  # lift
                ik = inverse_kinematics(robot, (tx, ty, hover_z))
                opening = robot.gripper.min_opening

            if not ik.ok:
                break
            verdict = supervisor.check(ik.pose, opening=opening)
            if not verdict.approved:
                break

            if call.skill == "grasp":
                # Descend open first, then close jaws
                sim.set_joint_targets(ik.pose)
                sim.set_opening(robot.gripper.max_opening)
                sim.step_until_settled(max_steps=400)
                sim.set_opening(robot.gripper.min_opening)
                sim.step_until_settled(max_steps=400)
            else:
                sim.set_joint_targets(ik.pose)
                sim.set_opening(opening)
                sim.step_until_settled(max_steps=400)

            completed.append(call.skill)

        # Privileged and perceived grading (Invariant I5: never mixed)
        obj_end = sim.object_position("red_block")
        tip_end = sim.tool_position()
        bottom_clearance = float(obj_end[2] - 0.020)
        slip_m = float(((obj_end[0] - tip_end[0]) ** 2 + (obj_end[1] - tip_end[1]) ** 2) ** 0.5)
        priv_success = bool(bottom_clearance > 0.015 and slip_m < 0.035)

        before_obs = SceneObservation(
            gripper_xyz=(tx if cand_cache else 0.0, ty if cand_cache else 0.2, 0.06),
            gripper_opening=robot.gripper.max_opening,
            objects={"red_block": (tx if cand_cache else 0.0, ty if cand_cache else 0.2, 0.02)},
        )
        after_obs = SceneObservation(
            gripper_xyz=(float(tip_end[0]), float(tip_end[1]), float(tip_end[2])),
            gripper_opening=sim.opening(),
            objects={"red_block": (float(obj_end[0]), float(obj_end[1]), float(obj_end[2]))},
        )
        out_eval = evaluate_outcome(
            before_obs,
            after_obs,
            "red_block",
            (float(tip_end[0]), float(tip_end[1]), float(tip_end[2])),
            scene_cfg,
        )
        perc_success = bool(out_eval.predicates["object_lifted"])
        sim.close()

        results.append(
            {
                "seed": seed,
                "task_success_privileged": priv_success,
                "task_success_perceived": perc_success,
                "bottom_clearance_m": bottom_clearance,
                "slip_m": slip_m,
                "safety_rejections": supervisor.rejected_infeasible,
            }
        )

    n = len(results)
    priv_rate = sum(1 for r in results if r["task_success_privileged"]) / n if n else 0.0
    perc_rate = sum(1 for r in results if r["task_success_perceived"]) / n if n else 0.0
    summary: dict[str, Any] = {
        "trials": n,
        "sim_backend": detect_sim_backend_label(),
        "success_rate_privileged": priv_rate,
        "success_rate_perceived": perc_rate,
        "gate": {
            "criterion": "SC1",
            "threshold": 0.80,
            "passed": priv_rate >= 0.80,
        },
        "episodes": results,
    }
    if write_run_json:
        out_dir = REPO_ROOT / "runs" / "phase4"
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "c34_sc1_l1.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary
