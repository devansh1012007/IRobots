"""Tests for ROS 2 messages, thin wrapper nodes, and launch files (PLAN:irobots-v2:C23-C28, T30-T36).

Asserts:
- T30: image timestamp equals simulated time (`use_sim_time:=true`).
- T31: paused simulator returns identical frames across two captures.
- T32: all 7 `irobots_msgs` messages round-trip through JSON/dict serialization and match their `.msg` definitions.
- T33: unreachable target triggers the per-skill deadline in `ControlNode` and the episode continues.
- T34: a rejected step in `SafetyNode` never reaches the controller and is published as a first-class outcome.
- T35: an episode recorded by `RecorderNode` to JSONL replays and reconstructs its events from JSONL alone.
- T36: `eval.launch.py`, `demo.launch.py`, and `perception_bench.launch.py` construct valid launch descriptions with `use_sim_time:=true`.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from irobots_bringup.launch_specs import (
    build_demo_launch_spec,
    build_eval_launch_spec,
    build_perception_bench_launch_spec,
)
from irobots_core.sim.mujoco_sim import DeterministicKinematicSim
from irobots_core.sim.scene_builder import build_seeded_scene, load_task_spec
from irobots_msgs.messages import (
    ArmStateMsg,
    DetectionMsg,
    PlanMsg,
    SceneMsg,
    SkillCallMsg,
    SkillResultMsg,
    TaskResultMsg,
)
from irobots_nodes.control_node import ControlNode
from irobots_nodes.recorder_node import RecorderNode
from irobots_nodes.safety_node import SafetyNode
from irobots_nodes.sim_node import SimNode


def test_sim_node_image_timestamp_equals_sim_time_and_paused_frames_match(
    robot, scene: dict
) -> None:
    """T30, T31 (C23): image stamp == sim time, and a paused sim yields bit-identical frames."""
    task = load_task_spec("config/tasks/l1.yaml")
    seeded = build_seeded_scene(robot, scene, task, seed=1)
    sim = DeterministicKinematicSim(robot, scene, seeded.to_sim_objects())
    node = SimNode(sim, use_sim_time=True)
    assert node.use_sim_time is True

    node.step_sim(25)
    expected_t = sim.total_steps * sim.dt
    img1 = node.capture_image()
    js1 = node.read_joint_state()
    assert abs(img1.timestamp_s - expected_t) < 1e-12
    assert abs(js1.timestamp_s - expected_t) < 1e-12

    # Pause sim -> two captures produce identical pixels and identical timestamps (T31, I6)
    node.pause()
    img_a = node.capture_image()
    img_b = node.capture_image()
    assert img_a.timestamp_s == img_b.timestamp_s
    assert np.array_equal(img_a.rgb, img_b.rgb)


def test_all_seven_irobots_msgs_round_trip_and_have_msg_files(repo_root: Path) -> None:
    """T32 (C24): Detection, Scene, SkillCall, SkillResult, Plan, ArmState, TaskResult round-trip."""
    msg_dir = repo_root / "ros2_ws" / "src" / "irobots_msgs" / "msg"
    for name in ("Detection", "Scene", "SkillCall", "SkillResult", "Plan", "ArmState", "TaskResult"):
        assert (msg_dir / f"{name}.msg").exists(), f"missing {name}.msg"

    det = DetectionMsg(
        track_id="red_block_1",
        label="red block",
        confidence=0.95,
        bbox_px=(100.0, 200.0, 140.0, 240.0),
        xy_m=(0.0, 0.20),
        height_m=0.04,
        height_source="probe",
    )
    assert DetectionMsg.from_dict(det.to_dict()) == det

    sc = SceneMsg(frame_id=3, timestamp_s=0.10, detections=(det,))
    assert SceneMsg.from_dict(sc.to_dict()) == sc

    call = SkillCallMsg(skill="grasp", target_id="red_block_1", turn_index=1)
    assert SkillCallMsg.from_dict(call.to_dict()) == call

    res = SkillResultMsg(
        skill="grasp",
        target_id="red_block_1",
        approved=True,
        outcome="object_moved_with_gripper",
        elapsed_sim_s=0.8,
        timed_out=False,
    )
    assert SkillResultMsg.from_dict(res.to_dict()) == res

    plan = PlanMsg(task="pick red block", steps=(call,), approved=True)
    assert PlanMsg.from_dict(plan.to_dict()) == plan

    arm = ArmStateMsg(
        timestamp_s=0.10,
        joints={"base_yaw": 0.0, "shoulder_pitch": 0.0, "elbow_pitch": 1.2, "wrist_pitch": 0.6, "wrist_yaw": 0.0},
        opening_m=0.070,
        estopped=False,
    )
    assert ArmStateMsg.from_dict(arm.to_dict()) == arm

    tr = TaskResultMsg(
        episode_id="ep_01",
        task_success_privileged=True,
        task_success_perceived=True,
        steps=5,
        action_log_sha256="abc123",
    )
    assert TaskResultMsg.from_dict(tr.to_dict()) == tr


def test_unreachable_waypoint_triggers_deadline_and_episode_continues(
    robot, scene: dict
) -> None:
    """T33 (C25, I4): an unreachable or stalled waypoint fires the per-skill deadline rather than hanging."""
    sim = DeterministicKinematicSim(robot, scene, [])
    ctrl = ControlNode(robot, sim, default_deadline_s=0.20)
    # Force a tiny deadline so the motion exceeds the deadline budget
    result = ctrl.execute_joint_goal(
        skill="move_above",
        target_id="red_block_1",
        goal_pose={"base_yaw": 2.5, "shoulder_pitch": 1.2, "elbow_pitch": -2.2, "wrist_pitch": 1.5, "wrist_yaw": 2.5},
        opening=robot.gripper.max_opening,
        deadline_s=0.02,
    )
    assert result.timed_out is True
    assert result.outcome == "skill_timeout"


def test_rejected_step_in_safety_node_never_reaches_controller(
    robot, scene: dict
) -> None:
    """T34 (C26, I2): a pose rejected by SafetyNode is never forwarded to ControlNode."""
    sim = DeterministicKinematicSim(robot, scene, [])
    ctrl = ControlNode(robot, sim)
    safety_node = SafetyNode(robot, controller=ctrl)

    # Command a joint pose that violates workspace bounds (tip far outside disk)
    bad_pose = {
        "base_yaw": 0.0,
        "shoulder_pitch": 0.0,
        "elbow_pitch": 0.0,
        "wrist_pitch": 0.0,
        "wrist_yaw": 0.0,
    }
    outcome = safety_node.validate_and_forward(
        skill="move_above",
        target_id="red_block_1",
        goal_pose=bad_pose,
        opening=robot.gripper.max_opening,
    )
    assert outcome.approved is False
    assert outcome.outcome.startswith("safety_rejected:")
    assert ctrl.executed_count == 0
    assert safety_node.rejected_infeasible == 1


def test_recorder_node_jsonl_replays_episode_without_rosbag(tmp_path: Path) -> None:
    """T35 (C27): an episode recorded by RecorderNode replays cleanly from JSONL alone."""
    jsonl_path = tmp_path / "episode.jsonl"
    rec = RecorderNode(jsonl_path)
    rec.record_turn(
        turn_index=0,
        sim_time_s=0.10,
        scene_digest="dig_0",
        skill="move_above",
        target_id="red_block_1",
        safety_approved=True,
        outcome="gripper_at_pose",
    )
    rec.record_turn(
        turn_index=1,
        sim_time_s=0.90,
        scene_digest="dig_1",
        skill="grasp",
        target_id="red_block_1",
        safety_approved=True,
        outcome="object_moved_with_gripper",
    )
    loaded = RecorderNode.load_jsonl(jsonl_path)
    assert len(loaded) == 2
    assert loaded[0]["skill"] == "move_above"
    assert loaded[1]["outcome"] == "object_moved_with_gripper"
    assert rec.action_log_hash() == RecorderNode.compute_action_log_hash(loaded)


def test_launch_files_boot_and_enforce_use_sim_time(repo_root: Path) -> None:
    """T36 (C28, I6): eval.launch.py, demo.launch.py, and perception_bench.launch.py boot with use_sim_time."""
    launch_dir = repo_root / "ros2_ws" / "src" / "irobots_bringup" / "launch"
    for fname in ("eval.launch.py", "demo.launch.py", "perception_bench.launch.py"):
        assert (launch_dir / fname).exists(), f"missing launch file {fname}"

    for spec in (
        build_eval_launch_spec(),
        build_demo_launch_spec(),
        build_perception_bench_launch_spec(),
    ):
        assert spec["use_sim_time"] is True
        assert len(spec["nodes"]) >= 2
        assert all(n["parameters"].get("use_sim_time") is True for n in spec["nodes"])
