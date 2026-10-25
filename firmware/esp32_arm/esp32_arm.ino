// Non-blocking ESP32 5-DOF arm controller speaking joint-space NDJSON over TCP.
// PLAN:irobots-v2:C48, D18, D19, D20
//
// Invariants enforced on device:
// - Receives JOINT ANGLES (degrees) + gripper opening (metres), never Cartesian targets.
// - Non-blocking millis() control loop (no blocking sleep calls) so e-stop acts within 1 cycle.
// - On e-stop or watchdog timeout: HOLDS the last commanded pose (keeps PWM active)
//   rather than dropping PWM to 0 as legacy V1_arm_controller_full_status.ino:313-323 did.

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "protocol_core.h"

static ArmProtocolCore g_core;
static unsigned long g_last_control_tick_ms = 0;
static const unsigned long CONTROL_PERIOD_MS = 10;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    g_last_control_tick_ms = millis();
}

void loop() {
    unsigned long now_ms = millis();
    if (now_ms - g_last_control_tick_ms >= CONTROL_PERIOD_MS) {
        g_last_control_tick_ms = now_ms;
        g_core.tick(now_ms);
    }
}
