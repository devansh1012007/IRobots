"""Tests for the closed loop, operator TUI, replay hash verifier, and e-stop (PLAN:irobots-v2:C36-C39, T46-T52).

Asserts:
- T46 (C36): a full L1 episode completes end-to-end (`observe -> plan/worker -> skill -> safety -> execute -> verify -> memory`).
- T47 (C36, I3): the loop never issues two skills for a single observation.
- T48, T49 (C37): `TUINode` supports the approve-then-autonomous (`plan -> run`) flow and handles keys `p s e r q` including the `e` e-stop interrupt.
- T50, T51 (C38): `replay_episode` reproduces the exact SHA-256 action-log hash of a recorded episode, and detects a tampered transcript.
- T52 (C39): `EStopController` aborts trajectory execution within one control cycle and holds the current pose.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from irobots_core.estop import EStopController
from irobots_core.loop import ClosedLoopRunner
from irobots_core.replay import TamperedTranscriptError, replay_episode
from irobots_nodes.tui_node import TUIMode, TUINode


def test_full_l1_episode_completes_and_enforces_one_skill_per_observation(
    robot, scene: dict, tmp_path: Path
) -> None:
    """T46, T47 (C36, I3): a full L1 episode completes and every skill has a fresh observation."""
    runner = ClosedLoopRunner(robot, scene)
    ep_path = tmp_path / "ep_l1.jsonl"
    result = runner.run_episode(
        task_path="config/tasks/l1.yaml",
        seed=0,
        jsonl_path=ep_path,
    )
    assert result.task_success_privileged is True
    assert result.task_success_perceived is True
    assert result.skills_executed == 3
    # T47 (I3): number of observations taken before skills equals skills_executed (never 2 skills per observation)
    assert result.observations_taken >= result.skills_executed
    assert result.max_skills_per_observation == 1
    assert ep_path.exists()


def test_tui_node_approve_then_autonomous_and_estop_key(
    robot, scene: dict
) -> None:
    """T48, T49 (C37): TUI starts in PLAN mode, approves on 'p' -> RUN, and 'e' triggers e-stop immediately."""
    tui = TUINode(robot, scene)
    assert tui.mode is TUIMode.PLAN
    assert tui.plan_approved is False

    # Key 'p' approves the plan and transitions to autonomous RUN mode (D11)
    tui.handle_key("p")
    assert tui.plan_approved is True
    assert tui.mode is TUIMode.RUN

    # Key 's' switches to single-step mode
    tui.handle_key("s")
    assert tui.mode is TUIMode.STEP

    # Key 'r' switches to replay mode
    tui.handle_key("r")
    assert tui.mode is TUIMode.REPLAY

    # Render rich dashboard text / layout without error
    rendered = tui.render_text()
    assert "SAFETY" in rendered
    assert "PLAN" in rendered

    # Key 'e' triggers e-stop (T49)
    tui.handle_key("e")
    assert tui.estop.triggered is True
    assert "E-STOP ACTIVE" in tui.render_text()


def test_replay_reproduces_action_log_hash_and_detects_tampering(
    robot, scene: dict, tmp_path: Path
) -> None:
    """T50, T51 (C38, D.7): replay reproduces the SHA-256 hash; tampering raises TamperedTranscriptError."""
    runner = ClosedLoopRunner(robot, scene)
    ep_path = tmp_path / "ep_replay.jsonl"
    res = runner.run_episode(
        task_path="config/tasks/l1.yaml",
        seed=2,
        jsonl_path=ep_path,
    )

    # T50: clean replay reproduces the exact action_log_sha256
    replay_res = replay_episode(ep_path, expected_hash=res.action_log_sha256)
    assert replay_res.reproduced is True
    assert replay_res.computed_sha256 == res.action_log_sha256

    # T51: tamper with one line in the transcript -> detected
    lines = ep_path.read_text(encoding="utf-8").splitlines()
    first = json.loads(lines[0])
    first["outcome"] = "forged_outcome"
    tampered_path = tmp_path / "ep_tampered.jsonl"
    tampered_path.write_text(
        "\n".join([json.dumps(first), *lines[1:]]) + "\n",
        encoding="utf-8",
    )
    with pytest.raises(TamperedTranscriptError, match="hash mismatch"):
        replay_episode(tampered_path, expected_hash=res.action_log_sha256)


def test_estop_aborts_within_one_control_cycle(robot, scene: dict) -> None:
    """T52 (C39, D18): triggering e-stop halts motion within 1 control step and holds pose."""
    from irobots_core.sim.mujoco_sim import DeterministicKinematicSim

    sim = DeterministicKinematicSim(robot, scene, [])
    sim.reset(dict(robot.home_pose), robot.gripper.max_opening)
    estop = EStopController()

    goal = {
        "base_yaw": 1.2,
        "shoulder_pitch": 0.5,
        "elbow_pitch": -0.5,
        "wrist_pitch": 0.3,
        "wrist_yaw": 0.8,
    }
    # Trigger e-stop before the control cycle steps
    estop.trigger(hold_pose=sim.joint_positions())
    steps_taken, aborted = estop.step_guarded(sim, goal_pose=goal, max_steps=100)
    assert aborted is True
    assert steps_taken <= 1
    assert sim.joint_positions() == pytest.approx(estop.held_pose, abs=1e-9)
