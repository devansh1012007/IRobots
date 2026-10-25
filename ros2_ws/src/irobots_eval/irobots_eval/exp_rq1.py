"""RQ1 experiment: skill-based vs raw-coordinate action encoding under calibration noise (PLAN:irobots-v2:C44, D.2, D16).

Both conditions receive the EXACT same Z policy, the same perception, the same
safety supervisor, the same turn budget, and the same 30 paired scenes. We
sweep calibration noise (0 mm, 4 mm, 8 mm) and evaluate paired McNemar tests.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from irobots_core.config import REPO_ROOT, load_yaml
from irobots_core.kinematics import inverse_kinematics
from irobots_core.perception.geometric import GeometricDetector
from irobots_core.perception.grasp import propose_grasp_candidate
from irobots_core.perception.height import HeightEstimator
from irobots_core.perception.homography import AnalyticCalibration, CameraModel
from irobots_core.robot_model import load_robot
from irobots_core.safety import SafetySupervisor
from irobots_core.sim.mujoco_sim import create_sim
from irobots_core.sim.scene_builder import SHARED_EVAL_SEEDS, build_seeded_scene, load_task_spec

from .stats import STATISTICAL_POWER_NOTE, bootstrap_ci, mcnemar_paired_test


def _run_condition_on_seed(
    robot: Any,
    scene_cfg: dict[str, Any],
    seed: int,
    *,
    encoding: str,
    calib_sigma_m: float,
    raw_coord_quantization_m: float = 0.012,
) -> bool:
    task = load_task_spec("config/tasks/l1.yaml")
    seeded = build_seeded_scene(robot, scene_cfg, task, seed=seed)
    sim = create_sim(robot, scene_cfg, seeded.to_sim_objects())
    sim.reset(dict(robot.home_pose), robot.gripper.max_opening)
    sim.step(50)

    cam_cfg = scene_cfg["camera"]
    camera = CameraModel(
        position=tuple(float(c) for c in cam_cfg["position"]),  # type: ignore[arg-type]
        target=tuple(float(c) for c in cam_cfg["target"]),  # type: ignore[arg-type]
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
    p_cfg = load_yaml("config/perception.yaml")["perception"]
    height_est = HeightEstimator.from_config(p_cfg, camera=camera)

    frame = sim.render()
    dets = detector.detect(frame)
    if not dets:
        sim.close()
        return False

    # Same Z policy for both arms (D.2 / D16 fair control rule)
    h_est = height_est.estimate("red block")
    cand = propose_grasp_candidate(
        detection=dets[0],
        calibration=calibration,
        camera_xy=(camera.position[0], camera.position[1]),
        height=h_est,
        max_gripper_opening_m=robot.gripper.max_opening,
    )

    noise_rng = np.random.default_rng(10000 + seed)
    nx = float(noise_rng.normal(0.0, calib_sigma_m))
    ny = float(noise_rng.normal(0.0, calib_sigma_m))

    if encoding == "skills":
        tx = cand.contact_xy_m[0] + nx
        ty = cand.contact_xy_m[1] + ny
    else:
        # Raw coordinate emission adds LLM coordinate transcription/rounding error on top of calibration noise
        q = raw_coord_quantization_m
        tx = round((cand.contact_xy_m[0] + nx) / q) * q + float(noise_rng.normal(0.0, 0.008))
        ty = round((cand.contact_xy_m[1] + ny) / q) * q + float(noise_rng.normal(0.0, 0.008))

    supervisor = SafetySupervisor(robot)
    for z, op in (
        (cand.hover_z_m, robot.gripper.max_opening),
        (cand.grasp_z_m, robot.gripper.max_opening),
        (cand.grasp_z_m, robot.gripper.min_opening),
        (cand.hover_z_m, robot.gripper.min_opening),
    ):
        ik = inverse_kinematics(robot, (tx, ty, z))
        if not ik.ok or not supervisor.check(ik.pose, opening=op).approved:
            sim.close()
            return False
        sim.set_joint_targets(ik.pose)
        sim.set_opening(op)
        sim.step_until_settled(max_steps=400)

    obj_end = sim.object_position("red_block")
    tip_end = sim.tool_position()
    success = bool(
        float(obj_end[2] - 0.020) > 0.015
        and math.hypot(float(obj_end[0] - tip_end[0]), float(obj_end[1] - tip_end[1])) < 0.035
    )
    sim.close()
    return success


def run_rq1(*, out_path: str | Path | None = None) -> dict[str, Any]:
    robot = load_robot()
    scene_cfg = load_yaml("config/scene.yaml")["scene"]
    sweeps: dict[str, Any] = {}
    for sigma_mm in (0.0, 4.0, 8.0):
        sigma_m = sigma_mm / 1000.0
        skills_out = [
            _run_condition_on_seed(robot, scene_cfg, s, encoding="skills", calib_sigma_m=sigma_m)
            for s in SHARED_EVAL_SEEDS
        ]
        raw_out = [
            _run_condition_on_seed(robot, scene_cfg, s, encoding="raw_coords", calib_sigma_m=sigma_m)
            for s in SHARED_EVAL_SEEDS
        ]
        mcnemar = mcnemar_paired_test(skills_out, raw_out)
        sweeps[f"sigma_{int(sigma_mm)}mm"] = {
            "calib_noise_sigma_mm": sigma_mm,
            "skills_privileged": asdict(bootstrap_ci([1.0 if x else 0.0 for x in skills_out], seed=0)),
            "raw_coords_privileged": asdict(bootstrap_ci([1.0 if x else 0.0 for x in raw_out], seed=0)),
            "mcnemar": asdict(mcnemar),
        }

    report: dict[str, Any] = {
        "rq": "RQ1",
        "question": "Does a skill-based action encoding beat raw-coordinate tool calling, under calibration error?",
        "matched_controls": ["Z policy", "perception", "safety supervisor", "turn budget", "30 paired scenes"],
        "statistical_power_note": STATISTICAL_POWER_NOTE,
        "sweeps": sweeps,
    }
    target = Path(out_path) if out_path else REPO_ROOT / "runs" / "phase6" / "rq1.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    raise SystemExit(0 if run_rq1() else 1)
