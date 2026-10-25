#!/usr/bin/env python
"""Generate firmware/esp32_arm/generated/robot_config.h from config/robot.yaml (PLAN:irobots-v2:C49, T56).

Enforces the single-source-of-truth rule: joint limits, gripper stroke, and
PCA9685 channel mappings are compiled into the firmware from ``config/robot.yaml``.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / "irobots_core"))

from irobots_core.robot_model import load_robot  # noqa: E402


def generate_header_text() -> str:
    robot = load_robot()
    lines = [
        "// AUTO-GENERATED FROM config/robot.yaml BY tools/gen_firmware_config.py",
        "// DO NOT EDIT BY HAND — PLAN:irobots-v2:C49",
        "#pragma once",
        "",
        f"#define PCA9685_ADDRESS 0x{robot.pca9685_address:02x}",
        f"#define SERVO_PULSE_MIN_US {robot.pulse_width_us[0]}",
        f"#define SERVO_PULSE_MAX_US {robot.pulse_width_us[1]}",
        "",
    ]
    for name, ch in robot.servo_channels.items():
        lines.append(f"#define CH_{name.upper()} {ch}")
    lines.append("")
    for name, j in robot.joints.items():
        u = name.upper()
        lines.append(f"#define {u}_MIN_DEG {j.min_deg:.1f}f")
        lines.append(f"#define {u}_MAX_DEG {j.max_deg:.1f}f")
    lines.append("")
    lines.append(f"#define GRIPPER_MIN_M {robot.gripper.min_opening:.4f}f")
    lines.append(f"#define GRIPPER_MAX_M {robot.gripper.max_opening:.4f}f")
    lines.append("#define WATCHDOG_TIMEOUT_MS 500")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    out_path = REPO_ROOT / "firmware" / "esp32_arm" / "generated" / "robot_config.h"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(generate_header_text(), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
