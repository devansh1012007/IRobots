"""Shared joint-space NDJSON wire protocol and conformance test suite (PLAN:irobots-v2:C51, T58).

Scenarios verified by :func:`run_conformance_suite`:
1. ``valid_move``: valid sequence ID and in-range joint angles -> ``ok=True, state=ACTIVE``.
2. ``out_of_limit``: joint angle or gripper opening outside ``config/robot.yaml`` limits -> ``ok=False, error=out_of_limit``.
3. ``sequence_gap``: non-monotonic or gap sequence ID -> ``ok=False, error=sequence_gap``.
4. ``watchdog_timeout``: elapsed time > 500 ms without command -> state transitions to ``WATCHDOG_HOLD`` holding last pose.
5. ``estop_hold``: e-stop command -> ``state=ESTOP_HOLD``, holds last pose, and rejects subsequent move commands until ``resume``.
6. ``malformed_line``: invalid JSON line -> ``ok=False, error=malformed_json``.
"""

from __future__ import annotations

import json
from typing import Any, Protocol


class ArmProtocolTarget(Protocol):
    def reset_protocol(self) -> None: ...
    def handle_line(self, line: str, now_ms: int = 0) -> dict[str, Any]: ...
    def advance_time_ms(self, now_ms: int) -> dict[str, Any]: ...


def run_conformance_suite(target: ArmProtocolTarget) -> dict[str, Any]:
    """Run all 6 protocol conformance scenarios against ``target`` (T57, T58)."""
    target.reset_protocol()
    results: dict[str, bool] = {}

    # 1. Malformed line
    r_mal = target.handle_line("{not valid json", now_ms=10)
    results["malformed_line"] = (r_mal.get("ok") is False and r_mal.get("error") == "malformed_json")

    # 2. Valid move (seq=1)
    cmd_move1 = json.dumps(
        {
            "cmd": "move",
            "seq": 1,
            "joints_deg": {
                "base_yaw": 15.0,
                "shoulder_pitch": 20.0,
                "elbow_pitch": -30.0,
                "wrist_pitch": 10.0,
                "wrist_yaw": 5.0,
            },
            "gripper_m": 0.045,
        }
    )
    r_move1 = target.handle_line(cmd_move1, now_ms=20)
    results["valid_move"] = (
        r_move1.get("ok") is True
        and r_move1.get("seq") == 1
        and r_move1.get("state") == "ACTIVE"
    )

    # 3. Sequence gap (seq=4 when expected is 2)
    cmd_gap = json.dumps(
        {
            "cmd": "move",
            "seq": 4,
            "joints_deg": {
                "base_yaw": 0.0,
                "shoulder_pitch": 0.0,
                "elbow_pitch": 0.0,
                "wrist_pitch": 0.0,
                "wrist_yaw": 0.0,
            },
            "gripper_m": 0.045,
        }
    )
    r_gap = target.handle_line(cmd_gap, now_ms=30)
    results["sequence_gap"] = (r_gap.get("ok") is False and r_gap.get("error") == "sequence_gap")

    # 4. Out-of-limit joint angle (seq=2, base_yaw=179.0 > 170.0)
    cmd_ool = json.dumps(
        {
            "cmd": "move",
            "seq": 2,
            "joints_deg": {
                "base_yaw": 179.0,
                "shoulder_pitch": 0.0,
                "elbow_pitch": 0.0,
                "wrist_pitch": 0.0,
                "wrist_yaw": 0.0,
            },
            "gripper_m": 0.045,
        }
    )
    r_ool = target.handle_line(cmd_ool, now_ms=40)
    results["out_of_limit"] = (r_ool.get("ok") is False and r_ool.get("error") == "out_of_limit")

    # 5. Watchdog timeout (>500 ms without a valid command holds last pose)
    r_wd = target.advance_time_ms(700)
    results["watchdog_timeout"] = (
        r_wd.get("state") == "WATCHDOG_HOLD"
        and abs(float(r_wd["held_joints_deg"]["base_yaw"]) - 15.0) < 1e-5
    )

    # 6. E-stop holds last pose and rejects moves until resume
    cmd_estop = json.dumps({"cmd": "estop", "seq": 2})
    r_estop = target.handle_line(cmd_estop, now_ms=710)
    cmd_after_estop = json.dumps(
        {
            "cmd": "move",
            "seq": 3,
            "joints_deg": {
                "base_yaw": 0.0,
                "shoulder_pitch": 0.0,
                "elbow_pitch": 0.0,
                "wrist_pitch": 0.0,
                "wrist_yaw": 0.0,
            },
            "gripper_m": 0.045,
        }
    )
    r_Blocked = target.handle_line(cmd_after_estop, now_ms=720)
    results["estop_hold"] = (
        r_estop.get("ok") is True
        and r_estop.get("state") == "ESTOP_HOLD"
        and r_Blocked.get("ok") is False
        and r_Blocked.get("error") == "estop_active"
        and abs(float(r_estop["held_joints_deg"]["base_yaw"]) - 15.0) < 1e-5
    )

    return {
        "scenarios": results,
        "all_passed": all(results.values()),
    }
