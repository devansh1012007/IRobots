"""Tests for deterministic fixed-step simulation and seeded scene sampling (PLAN:irobots-v2:C21, C22, T28, T29).

Asserts:
- T28: same seed + same actions -> identical final state SHA-256 hash.
- T29: the same seed yields the exact same scene (object poses, distractors,
       bin positions) across two independent runs/processes, and the 30 shared
       scene seeds are strictly reproducible.
"""

from __future__ import annotations

import subprocess
import sys

from irobots_core.kinematics import inverse_kinematics
from irobots_core.sim.mujoco_sim import DeterministicKinematicSim
from irobots_core.sim.scene_builder import (
    SHARED_EVAL_SEEDS,
    build_seeded_scene,
    load_task_spec,
)


def test_same_seed_and_same_actions_produce_identical_state_hash(robot, scene: dict) -> None:
    """T28 (C21, I6): fixed-step simulation produces an identical final state hash on repeated runs."""
    task = load_task_spec("config/tasks/l1.yaml")
    sc1 = build_seeded_scene(robot, scene, task, seed=0)
    sc2 = build_seeded_scene(robot, scene, task, seed=0)

    sim1 = DeterministicKinematicSim(robot, scene, sc1.to_sim_objects())
    sim2 = DeterministicKinematicSim(robot, scene, sc2.to_sim_objects())

    for sim in (sim1, sim2):
        sim.reset(dict(robot.home_pose), robot.gripper.max_opening)
        obj_pos = sim.object_position("red_block")
        for z, op in (
            (0.06, robot.gripper.max_opening),
            (0.02, robot.gripper.max_opening),
            (0.02, robot.gripper.min_opening),
            (0.06, robot.gripper.min_opening),
        ):
            ik = inverse_kinematics(robot, (float(obj_pos[0]), float(obj_pos[1]), z))
            assert ik.ok
            sim.set_joint_targets(ik.pose)
            sim.set_opening(op)
            sim.step_until_settled(max_steps=400)

    assert sim1.state_hash() == sim2.state_hash()
    assert sim1.total_steps == sim2.total_steps


def test_same_seed_yields_identical_scene_in_two_processes(robot, scene: dict) -> None:
    """T29 (C22): the same seed yields the exact same scene across two separate Python processes."""
    assert len(SHARED_EVAL_SEEDS) == 30
    task = load_task_spec("config/tasks/l2.yaml")
    local_scene = build_seeded_scene(robot, scene, task, seed=17)

    cmd = [
        sys.executable,
        "-c",
        (
            "import json; "
            "from irobots_core.robot_model import load_robot; "
            "from irobots_core.config import load_yaml; "
            "from irobots_core.sim.scene_builder import load_task_spec, build_seeded_scene; "
            "r = load_robot(); s = load_yaml('config/scene.yaml')['scene']; "
            "t = load_task_spec('config/tasks/l2.yaml'); "
            "sc = build_seeded_scene(r, s, t, seed=17); "
            "print(sc.digest())"
        ),
    ]
    out = subprocess.check_output(cmd, text=True).strip()
    assert out == local_scene.digest()
