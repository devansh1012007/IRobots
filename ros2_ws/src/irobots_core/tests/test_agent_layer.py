"""Tests for LLM client, tool-call schema validation, planner, worker, memory, and scripted policy (PLAN:irobots-v2:C29-C34, T37-T45, SC1).

Asserts:
- T37 (C29): all three LLM profiles (`openrouter`, `lmstudio`, `scripted`) answer the same prompt shape and log a SHA-256 request/response hash.
- T38 (C30): malformed, partial, non-reasoning, and index-1 tool calls are handled without crashing.
- T39 (C30): after one repair retry fails, fallback to the scripted policy is engaged and logged.
- T40 (C31): a plan naming an object absent from the scene is rejected at approval.
- T41 (C32): the worker returns exactly one skill call per turn (never chains skills, I3).
- T42 (C32): repeat detection (same skill + target twice in a row) escalates for replanning.
- T43 (C32): unknown target ID is refused with `unknown_target`.
- T44 (C33): episode memory survives across turns within an episode and resets cleanly across episodes.
- T45 (C34, SC1): scripted reference policy reaches >= 80% success on L1 across the 30 seeded scenes.
"""

from __future__ import annotations

import pytest

from irobots_core.agent.memory import EpisodeMemory
from irobots_core.agent.planner import PlanApprovalError, TaskPlanner
from irobots_core.agent.policy import ScriptedReferencePolicy, evaluate_scripted_policy_l1
from irobots_core.agent.worker import RepeatActionEscalation, WorkerAgent
from irobots_core.llm.client import LLMClient, LLMProfile
from irobots_core.llm.schema import parse_and_validate_tool_call
from irobots_core.skills import SkillCall, SkillValidationError


def test_all_three_llm_profiles_answer_same_prompt_shape() -> None:
    """T37 (C29): openrouter, lmstudio, and scripted profiles answer the same prompt shape with hashes."""
    messages = [{"role": "user", "content": "Pick up red_block_1"}]
    available_ids = ["red_block_1"]

    def fake_transport(profile: LLMProfile, payload: dict) -> dict:
        return {
            "id": f"resp_{profile.value}",
            "model": payload.get("model", "test-model"),
            "system_fingerprint": "fp_1234",
            "choices": [
                {
                    "message": {
                        "role": "assistant",
                        "content": None,
                        "tool_calls": [
                            {
                                "index": 0,
                                "type": "function",
                                "function": {
                                    "name": "move_above",
                                    "arguments": '{"target_id": "red_block_1"}',
                                },
                            }
                        ],
                    }
                }
            ],
        }

    for profile in (LLMProfile.OPENROUTER, LLMProfile.LMSTUDIO, LLMProfile.SCRIPTED):
        client = LLMClient(profile=profile, transport=fake_transport)
        resp = client.chat_tool_call(messages=messages, available_target_ids=available_ids)
        assert resp.skill_call.skill == "move_above"
        assert resp.skill_call.target_id == "red_block_1"
        assert len(resp.request_sha256) == 64
        assert len(resp.response_sha256) == 64


def test_malformed_partial_and_index1_tool_calls_and_fallback() -> None:
    """T38, T39 (C30): index-1 chunks, missing reasoning tokens, repair retry, and scripted fallback."""
    known_ids = {"red_block_1"}
    fallback = SkillCall(skill="move_above", target_id="red_block_1")

    # 1. Model emits tool_call at index=1 with no reasoning tokens (legacy crashed with UnboundLocalError)
    raw_idx1 = {
        "choices": [
            {
                "message": {
                    "content": None,
                    "tool_calls": [
                        {"index": 0, "function": {"name": "", "arguments": ""}},
                        {"index": 1, "function": {"name": "grasp", "arguments": '{"target_id": "red_block_1"}'}},
                    ],
                }
            }
        ]
    }
    res_idx1 = parse_and_validate_tool_call(raw_idx1, known_ids, fallback_call=fallback)
    assert res_idx1.call == SkillCall(skill="grasp", target_id="red_block_1")
    assert res_idx1.used_fallback is False

    # 2. Malformed JSON repaired on retry -> succeeds without fallback
    bad_raw = {
        "choices": [
            {"message": {"tool_calls": [{"index": 0, "function": {"name": "lift", "arguments": "{bad json"}}]}}
        ]
    }
    good_retry = {
        "choices": [
            {
                "message": {
                    "tool_calls": [
                        {"index": 0, "function": {"name": "lift", "arguments": '{"target_id": "red_block_1"}'}}
                    ]
                }
            }
        ]
    }
    res_repaired = parse_and_validate_tool_call(
        bad_raw,
        known_ids,
        repair_fn=lambda _err: good_retry,
        fallback_call=fallback,
    )
    assert res_repaired.call == SkillCall(skill="lift", target_id="red_block_1")
    assert res_repaired.repaired is True
    assert res_repaired.used_fallback is False

    # 3. Malformed even after 1 repair retry -> engages scripted fallback and logs it (T39)
    res_fb = parse_and_validate_tool_call(
        bad_raw,
        known_ids,
        repair_fn=lambda _err: bad_raw,
        fallback_call=fallback,
    )
    assert res_fb.call == fallback
    assert res_fb.used_fallback is True
    assert res_fb.error_log != ""


def test_planner_rejects_plan_referencing_absent_object() -> None:
    """T40 (C31): a plan naming an object not in the current scene is rejected at approval."""
    planner = TaskPlanner()
    proposed = [
        SkillCall(skill="move_above", target_id="green_block_99"),
        SkillCall(skill="grasp", target_id="green_block_99"),
    ]
    with pytest.raises(PlanApprovalError, match="green_block_99"):
        planner.validate_and_approve(proposed, scene_target_ids={"red_block_1"})

    valid_plan = [
        SkillCall(skill="move_above", target_id="red_block_1"),
        SkillCall(skill="grasp", target_id="red_block_1"),
        SkillCall(skill="lift", target_id="red_block_1"),
    ]
    approved = planner.validate_and_approve(valid_plan, scene_target_ids={"red_block_1"})
    assert len(approved) == 3


def test_worker_enforces_one_call_repeat_detection_and_unknown_id() -> None:
    """T41, T42, T43 (C32, I3): one call per turn, repeat detection escalates, unknown ID refused."""
    mem = EpisodeMemory(episode_id="ep_1")
    worker = WorkerAgent()

    # T41: if a candidate list of multiple skills is passed, worker emits strictly ONE skill call (I3)
    call1 = worker.select_next_skill(
        plan=[
            SkillCall(skill="move_above", target_id="red_block_1"),
            SkillCall(skill="grasp", target_id="red_block_1"),
        ],
        scene_target_ids={"red_block_1"},
        memory=mem,
    )
    assert isinstance(call1, SkillCall)
    assert call1.skill == "move_above"

    # Record two consecutive failed attempts of the exact same (skill, target_id) -> T42 repeat escalation
    mem.record_step(call1, outcome="no_target", success=False)
    mem.record_step(call1, outcome="no_target", success=False)
    with pytest.raises(RepeatActionEscalation, match="repeated"):
        worker.select_next_skill(
            plan=[call1],
            scene_target_ids={"red_block_1"},
            memory=mem,
        )

    # T43: unknown target ID in scene is refused
    mem.reset("ep_2")
    with pytest.raises(SkillValidationError, match="unknown_target"):
        worker.select_next_skill(
            plan=[SkillCall(skill="grasp", target_id="missing_id")],
            scene_target_ids={"red_block_1"},
            memory=mem,
        )


def test_episode_memory_survives_turns_and_resets_across_episodes() -> None:
    """T44 (C33): memory persists across turns within an episode and clears on new episode."""
    mem = EpisodeMemory(episode_id="ep_a")
    c1 = SkillCall(skill="move_above", target_id="red_block_1")
    c2 = SkillCall(skill="grasp", target_id="red_block_1")
    mem.record_step(c1, outcome="gripper_at_pose", success=True)
    mem.record_step(c2, outcome="object_knocked_away", success=False, hypothesis="cube shifted right")
    assert len(mem.history) == 2
    assert len(mem.failures()) == 1
    assert mem.current_hypothesis == "cube shifted right"

    # Resetting for the next episode clears all state so nothing leaks across episodes
    mem.reset("ep_b")
    assert mem.episode_id == "ep_b"
    assert len(mem.history) == 0
    assert mem.current_hypothesis == ""


def test_scripted_reference_policy_reaches_at_least_80_percent_on_l1(robot, scene: dict) -> None:
    """T45 (C34, SC1): scripted reference policy reaches >= 80% success on 30 seeded L1 scenes."""
    policy = ScriptedReferencePolicy()
    summary = evaluate_scripted_policy_l1(robot, scene, policy=policy, seeds=range(30))
    assert summary["trials"] == 30
    assert summary["success_rate_privileged"] >= 0.80, (
        f"SC1 failed: scripted policy reached {summary['success_rate_privileged']:.1%} < 80%"
    )
