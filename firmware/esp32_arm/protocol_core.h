// Pure C++ protocol & state machine core shared by esp32_arm.ino and host conformance tests.
// PLAN:irobots-v2:C48, C51
#pragma once

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include "generated/robot_config.h"

enum class ArmFirmwareState {
    IDLE = 0,
    ACTIVE = 1,
    WATCHDOG_HOLD = 2,
    ESTOP_HOLD = 3
};

struct ArmProtocolCore {
    ArmFirmwareState state = ArmFirmwareState::IDLE;
    int last_seq = 0;
    unsigned long last_cmd_ms = 0;
    float joints_deg[5] = {0.0f, 0.0f, 68.75f, 34.38f, 0.0f};
    float gripper_m = 0.045f;

    static const char* state_name(ArmFirmwareState s) {
        switch (s) {
            case ArmFirmwareState::IDLE: return "IDLE";
            case ArmFirmwareState::ACTIVE: return "ACTIVE";
            case ArmFirmwareState::WATCHDOG_HOLD: return "WATCHDOG_HOLD";
            case ArmFirmwareState::ESTOP_HOLD: return "ESTOP_HOLD";
        }
        return "IDLE";
    }

    void tick(unsigned long now_ms) {
        if (state == ArmFirmwareState::ACTIVE && (now_ms - last_cmd_ms) > WATCHDOG_TIMEOUT_MS) {
            // Watchdog holds the current pose rather than going limp
            state = ArmFirmwareState::WATCHDOG_HOLD;
        }
    }

    bool check_limits(float b, float s, float e, float wp, float wy, float g) const {
        if (b < BASE_YAW_MIN_DEG || b > BASE_YAW_MAX_DEG) return false;
        if (s < SHOULDER_PITCH_MIN_DEG || s > SHOULDER_PITCH_MAX_DEG) return false;
        if (e < ELBOW_PITCH_MIN_DEG || e > ELBOW_PITCH_MAX_DEG) return false;
        if (wp < WRIST_PITCH_MIN_DEG || wp > WRIST_PITCH_MAX_DEG) return false;
        if (wy < WRIST_YAW_MIN_DEG || wy > WRIST_YAW_MAX_DEG) return false;
        if (g < GRIPPER_MIN_M - 1e-5f || g > GRIPPER_MAX_M + 1e-5f) return false;
        return true;
    }
};
