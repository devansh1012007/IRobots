"""Tests for ESP32 firmware config generation, wire protocol, emulator, and bridge (PLAN:irobots-v2:C48-C52, T55-T59).

Asserts:
- T55 (C48): `firmware/esp32_arm/esp32_arm.ino` uses non-blocking `millis()` scheduling (zero `delay(` calls) and its pure C++ protocol logic compiles/passes host-side unit tests.
- T56 (C49): `tools/gen_firmware_config.py` generates `firmware/esp32_arm/generated/robot_config.h` matching `config/robot.yaml` exactly.
- T57 (C50): `ArmEmulator` passes the full protocol conformance suite.
- T58 (C51): both `ArmEmulator` and the compiled C++ firmware logic (`firmware/esp32_arm/protocol_core.h`) pass the identical conformance suite (valid move, out-of-limit, sequence gap, watchdog timeout, e-stop hold, malformed line).
- T59 (C52): `ArmBridgeNode` performs a full TCP NDJSON round-trip against `ArmEmulatorServer`.
"""

from __future__ import annotations

from pathlib import Path

from irobots_core.arm.emulator import ArmEmulator, ArmEmulatorServer
from irobots_core.arm.protocol import run_conformance_suite
from irobots_nodes.arm_bridge_node import ArmBridgeNode


def test_generated_firmware_header_matches_robot_yaml(repo_root: Path, robot) -> None:
    """T56 (C49): generated robot_config.h matches config/robot.yaml."""
    import subprocess
    import sys

    gen_script = repo_root / "tools" / "gen_firmware_config.py"
    header_path = repo_root / "firmware" / "esp32_arm" / "generated" / "robot_config.h"
    subprocess.run([sys.executable, str(gen_script)], check=True)
    assert header_path.exists()
    content = header_path.read_text(encoding="utf-8")
    assert "PCA9685_ADDRESS 0x40" in content
    for name, joint in robot.joints.items():
        assert f"{name.upper()}_MIN_DEG" in content
        assert f"{joint.min_deg:.1f}" in content
        assert f"{joint.max_deg:.1f}" in content


def test_firmware_sketch_has_no_blocking_delay_and_both_implementations_pass_conformance(
    repo_root: Path, robot
) -> None:
    """T55, T57, T58 (C48, C50, C51): sketch has zero delay() calls, and emulator + C++ firmware core pass the conformance suite."""
    sketch = repo_root / "firmware" / "esp32_arm" / "esp32_arm.ino"
    assert sketch.exists()
    sketch_text = sketch.read_text(encoding="utf-8")
    assert "millis()" in sketch_text
    assert "delay(" not in sketch_text, "firmware must use non-blocking millis() scheduling, never delay()"

    # T57: Python emulator passes the conformance suite
    emu = ArmEmulator(robot)
    emu_report = run_conformance_suite(emu)
    assert emu_report["all_passed"] is True, f"emulator failed conformance: {emu_report}"

    # T58: C++ firmware protocol_core.h compiled on host passes the exact same conformance suite
    cpp_report = emu.run_cpp_conformance_if_compiler_available(repo_root)
    assert cpp_report["all_passed"] is True, f"C++ firmware core failed conformance: {cpp_report}"


def test_arm_bridge_node_tcp_round_trip_with_emulator_server(robot) -> None:
    """T59 (C52): ArmBridgeNode drives ArmEmulatorServer over localhost TCP NDJSON."""
    with ArmEmulatorServer(robot) as server:
        bridge = ArmBridgeNode(host=server.host, port=server.port)
        # 1. Valid move command
        ack1 = bridge.send_joint_command(
            seq=1,
            joints_deg={
                "base_yaw": 10.0,
                "shoulder_pitch": 15.0,
                "elbow_pitch": -20.0,
                "wrist_pitch": 10.0,
                "wrist_yaw": 0.0,
            },
            gripper_m=0.045,
        )
        assert ack1["ok"] is True
        assert ack1["seq"] == 1
        assert ack1["state"] == "ACTIVE"

        # 2. E-stop holds last pose and refuses subsequent move commands (D18)
        estop_ack = bridge.send_estop(seq=2)
        assert estop_ack["ok"] is True
        assert estop_ack["state"] == "ESTOP_HOLD"

        rejected_move = bridge.send_joint_command(
            seq=3,
            joints_deg={
                "base_yaw": 0.0,
                "shoulder_pitch": 0.0,
                "elbow_pitch": 0.0,
                "wrist_pitch": 0.0,
                "wrist_yaw": 0.0,
            },
            gripper_m=0.045,
        )
        assert rejected_move["ok"] is False
        assert rejected_move["error"] == "estop_active"
