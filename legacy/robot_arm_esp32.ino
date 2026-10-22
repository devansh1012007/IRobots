#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LedControl.h>   // Install via Library Manager: "LedControl" by Eberhard Fahle

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// =========================================================
// PORTED FROM ARDUINO UNO/NANO TO ESP32
// Changes made (read before wiring):
//  - I2C (PCA9685): ESP32 default is SDA=21, SCL=22. Wire.begin() uses these
//    automatically. If your board/wiring differs, call
//    Wire.begin(SDA_PIN, SCL_PIN) instead in setup().
//  - E-STOP pin moved to GPIO 27 (was Uno pin 2). Avoid strapping pins
//    (0, 2, 12, 15) and input-only pins (34-39, no internal pulldown) for
//    this signal since it needs attachInterrupt + pinMode INPUT.
//  - MAX7219 dot matrix pins moved to GPIO 23 (DIN), 5 (CS), 18 (CLK) —
//    these are bit-banged by the LedControl library, so any free GPIO works.
//  - BUZZER and RGB LED moved off Uno pins 3/4/5/6 (some of those clash with
//    the new matrix wiring above) to GPIO 25 (buzzer), 26/32/33 (R/G/B).
//  - analogWrite()/tone() are reimplemented on top of the ESP32 LEDC
//    peripheral (setStatusColor() and toneESP32()/noToneESP32()), because
//    older esp32 Arduino cores don't support analogWrite()/tone() the same
//    way Uno does. Function names in the rest of the sketch are unchanged
//    (setStatusColor, tone, noTone) via thin wrapper macros/functions below.
//  - ISR now carries IRAM_ATTR (required on ESP32; interrupt code must
//    live in IRAM, not flash).
//  - eStopActive is declared volatile as before; ESP32 is dual-core so this
//    matters just as much (if not more) than on Uno.
// =========================================================

// =========================================================
// SERVO CHANNEL DEFINITIONS (0-15 on the PCA9685 board) — UNCHANGED
// =========================================================
// --- ARM 1 (existing) ---
#define BASE        0
#define SHOULDER    1
#define ELBOW       2
#define WRIST_ROT   3
#define WRIST_PITCH 4
#define GRIPPER     5

// --- ARM 2 (second 3-servo arm) ---
#define BASE2     6
#define SHOULDER2 7
#define ELBOW2    8

// Pulse Width Limits (Microseconds) — UNCHANGED
#define SERVOMIN  500
#define SERVOMAX  2400

// =========================================================
// I2C PINS (ESP32) — set to your actual wiring if not using defaults
// =========================================================
#define I2C_SDA 21
#define I2C_SCL 22

// =========================================================
// 8x8 DOT MATRIX (MAX7219) — ESP32 GPIOs (bit-banged, any free pin works)
// =========================================================
#define MATRIX_DIN 23
#define MATRIX_CS  5
#define MATRIX_CLK 18
LedControl lc = LedControl(MATRIX_DIN, MATRIX_CLK, MATRIX_CS, 1); // 1 = one 8x8 module

// =========================================================
// EMERGENCY STOP BUTTON — external pulldown circuit:
//   unpressed = LOW, pressed = HIGH (same electrical behavior as before)
// =========================================================
#define ESTOP_PIN 27
volatile bool eStopActive = false;

// =========================================================
// BUZZER — driven via LEDC (ESP32 has no native tone() on all cores)
// =========================================================
#define BUZZER_PIN 25
#define BUZZER_LEDC_CHANNEL 4   // must not collide with RGB channels below

// =========================================================
// RGB STATUS LED — driven via LEDC PWM (replaces analogWrite)
// =========================================================
#define RGB_RED_PIN   26
#define RGB_GREEN_PIN 32
#define RGB_BLUE_PIN  33
#define RGB_RED_LEDC_CHANNEL   1
#define RGB_GREEN_LEDC_CHANNEL 2
#define RGB_BLUE_LEDC_CHANNEL  3
#define LEDC_FREQ_PWM   5000
#define LEDC_RES_PWM_BITS 8

// =========================================================
// 8x8 BITMAPS (one byte per row, top row first) — UNCHANGED
// =========================================================
byte NUM_3[8] = {
  B00111100,
  B01000010,
  B00000010,
  B00011100,
  B00000010,
  B00000010,
  B01000010,
  B00111100
};
byte NUM_2[8] = {
  B00111100,
  B01000010,
  B00000010,
  B00000100,
  B00001000,
  B00010000,
  B00100000,
  B01111110
};
byte NUM_1[8] = {
  B00001000,
  B00011000,
  B00101000,
  B00001000,
  B00001000,
  B00001000,
  B00001000,
  B00111110
};
byte CHECKMARK[8] = {
  B00000000,
  B00000001,
  B00000010,
  B00000100,
  B10001000,
  B01010000,
  B00100000,
  B00000000
};
byte XMARK[8] = {
  B10000001,
  B01000010,
  B00100100,
  B00011000,
  B00011000,
  B00100100,
  B01000010,
  B10000001
};

int processingFrame = 0; // used by the scanning-bar "processing" animation

void setup() {
  Serial.begin(115200); // ESP32 default monitor speed; change if you prefer 9600

  // --- I2C / Servo driver ---
  Wire.begin(I2C_SDA, I2C_SCL);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Standard 50Hz for analog servos
  delay(10);

  // --- Dot matrix ---
  lc.shutdown(0, false);   // wake the display up
  lc.setIntensity(0, 8);   // brightness 0-15
  lc.clearDisplay(0);

  // --- RGB status LED (LEDC setup) ---
  ledcSetup(RGB_RED_LEDC_CHANNEL,   LEDC_FREQ_PWM, LEDC_RES_PWM_BITS);
  ledcSetup(RGB_GREEN_LEDC_CHANNEL, LEDC_FREQ_PWM, LEDC_RES_PWM_BITS);
  ledcSetup(RGB_BLUE_LEDC_CHANNEL,  LEDC_FREQ_PWM, LEDC_RES_PWM_BITS);
  ledcAttachPin(RGB_RED_PIN,   RGB_RED_LEDC_CHANNEL);
  ledcAttachPin(RGB_GREEN_PIN, RGB_GREEN_LEDC_CHANNEL);
  ledcAttachPin(RGB_BLUE_PIN,  RGB_BLUE_LEDC_CHANNEL);
  setStatusColor(0, 0, 0);

  // --- Buzzer (LEDC setup, channel starts idle) ---
  ledcSetup(BUZZER_LEDC_CHANNEL, 2000, 10); // freq gets overwritten per-tone() call
  ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);

  // --- Emergency stop ---
  pinMode(ESTOP_PIN, INPUT); // external pulldown resistor, same wiring as before
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), eStopISR, RISING);

  // --- Initialize ARM 1 to neutral ---
  //setServoAngle(BASE, 10);         delay(500);
  setServoAngle(SHOULDER, 100);     delay(500);
  setServoAngle(ELBOW, 97);        delay(500);
  setServoAngle(WRIST_ROT, 100);     delay(500);
  // setServoAngle(WRIST_PITCH, 45);  delay(500);
  setServoAngle(GRIPPER, 90);      delay(500); // Open/Neutral // good enough open // 40- 50 for close

  // --- Initialize ARM 2 to neutral ---
  //setServoAngle(BASE2, 90);        delay(200);
  //setServoAngle(SHOULDER2, 90);    delay(200);
  //setServoAngle(ELBOW2, 90);       delay(200);
}

void loop() {
  if (checkEStop()) return; // don't start a new cycle mid-stop

  runCountdown();
  if (checkEStop()) return;

  setStatusColor(0, 255, 0); // green: motors running
  Serial.println("Processing...");

  // --- Sweep ARM 1 and ARM 2 to their first test position ---
  //moveJointAnimated(BASE, 20, 1000);
  //moveJointAnimated(BASE, 10, 1000);
  if (checkEStop()) return;
  moveJointAnimated(SHOULDER, 96, 1000);
  moveJointAnimated(SHOULDER, 100, 1000);
  if (checkEStop()) return;
  // moveJointAnimated(ELBOW, 100, 1000);
  // moveJointAnimated(ELBOW, 97, 1000);
  moveJointAnimated(WRIST_ROT, 80, 500);
  moveJointAnimated(WRIST_ROT, 120, 500);
  moveJointAnimated(WRIST_ROT, 100, 500);
  moveJointAnimated(WRIST_PITCH, 100, 500);
  moveJointAnimated(WRIST_PITCH, 20, 500);
  moveJointAnimated(GRIPPER, 40, 500); // Close gripper
  moveJointAnimated(GRIPPER, 90, 500); // Close gripper
  if (checkEStop()) return;

  // moveJointAnimated(BASE2, 45, 1000);
  // if (checkEStop()) return;
  // moveJointAnimated(SHOULDER2, 60, 1000);
  // if (checkEStop()) return;
  // moveJointAnimated(ELBOW2, 50, 1000);
  // if (checkEStop()) return;

  delay(500);

  // --- Return ARM 1 and ARM 2 to their second test position ---
  // moveJointAnimated(BASE, 95, 1000);
  // if (checkEStop()) return;
  // moveJointAnimated(SHOULDER, 100, 1000);
  // if (checkEStop()) return;
  // moveJointAnimated(ELBOW, 95, 1000);
  // moveJointAnimated(WRIST_ROT, 100, 500);
  // if (checkEStop()) return;
  // moveJointAnimated(WRIST_PITCH, 160, 500);
  // if (checkEStop()) return;
  // moveJointAnimated(GRIPPER, 50, 500); // Open gripper
  // if (checkEStop()) return;

  // moveJointAnimated(BASE2, 95, 1000);
  // if (checkEStop()) return;
  // moveJointAnimated(SHOULDER2, 100, 1000);
  // if (checkEStop()) return;
  // moveJointAnimated(ELBOW2, 90, 1000);
  // if (checkEStop()) return;

  // delay(500);

  // --- Cycle complete ---
  Serial.println("Tasks completed.");
  lc.clearDisplay(0);
  displayBitmap(CHECKMARK);
  delay(1000);
  lc.clearDisplay(0);
}

// =========================================================
// COUNTDOWN — yellow status color, beeps, 3-2-1 on the matrix
// =========================================================
void runCountdown() {
  setStatusColor(255, 255, 0); // yellow

  displayBitmap(NUM_3);
  toneESP32(BUZZER_PIN, 1000, 150);
  delay(1000);
  if (checkEStop()) return;

  displayBitmap(NUM_2);
  toneESP32(BUZZER_PIN, 1000, 150);
  delay(1000);
  if (checkEStop()) return;

  displayBitmap(NUM_1);
  toneESP32(BUZZER_PIN, 1000, 150);
  delay(1000);
  if (checkEStop()) return;

  toneESP32(BUZZER_PIN, 1800, 300); // longer "go" beep
  lc.clearDisplay(0);
  delay(300);
}

// =========================================================
// SERVO HELPERS — UNCHANGED (Adafruit PWM library is portable)
// =========================================================
void setServoAngle(uint8_t channel, int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  int pulse_us = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  int pwm_value = (int)((double)pulse_us * 0.2048);

  pwm.setPWM(channel, 0, pwm_value);
}

void moveJoint(uint8_t channel, int targetAngle, int delayMs) {
  if (eStopActive) return;
  setServoAngle(channel, targetAngle);
  delay(delayMs);
}

// Same as moveJoint, but also advances the "processing" scanning-bar
// animation on the dot matrix so it visibly updates while a motor moves.
void moveJointAnimated(uint8_t channel, int targetAngle, int delayMs) {
  if (eStopActive) return;
  showProcessingFrame();
  setServoAngle(channel, targetAngle);
  delay(delayMs);
}

void showProcessingFrame() {
  for (int row = 0; row < 8; row++) {
    lc.setRow(0, row, (row == processingFrame % 8) ? 0xFF : 0x00);
  }
  processingFrame++;
}

void displayBitmap(byte bmp[8]) {
  for (int row = 0; row < 8; row++) {
    lc.setRow(0, row, bmp[row]);
  }
}

// =========================================================
// RGB LED — reimplemented on ESP32 LEDC (replaces analogWrite)
// Kept the same function signature/name so the rest of the sketch
// doesn't need to change.
// =========================================================
void setStatusColor(int r, int g, int b) {
  ledcWrite(RGB_RED_LEDC_CHANNEL,   constrain(r, 0, 255));
  ledcWrite(RGB_GREEN_LEDC_CHANNEL, constrain(g, 0, 255));
  ledcWrite(RGB_BLUE_LEDC_CHANNEL,  constrain(b, 0, 255));
}

// =========================================================
// BUZZER — reimplemented on ESP32 LEDC (replaces tone()/noTone()).
// Named toneESP32/noToneESP32 to avoid silently colliding with any
// core-provided tone() on newer esp32 Arduino cores.
// =========================================================
void toneESP32(uint8_t pin, unsigned int frequency, unsigned long duration) {
  ledcWriteTone(BUZZER_LEDC_CHANNEL, frequency);
  delay(duration);
  ledcWriteTone(BUZZER_LEDC_CHANNEL, 0); // stop after the requested duration,
                                          // matching Uno tone()'s non-blocking-
                                          // but-timed behavior closely enough
                                          // for this sketch's fixed delay() calls
}

void noToneESP32(uint8_t pin) {
  ledcWriteTone(BUZZER_LEDC_CHANNEL, 0);
}

// =========================================================
// EMERGENCY STOP
// =========================================================

// ISR must stay short and in IRAM on ESP32 — just set a flag.
void IRAM_ATTR eStopISR() {
  eStopActive = true;
}

void haltAllServos() {
  pwm.setPWM(BASE, 0, 0);
  pwm.setPWM(SHOULDER, 0, 0);
  pwm.setPWM(ELBOW, 0, 0);
  pwm.setPWM(WRIST_ROT, 0, 0);
  pwm.setPWM(WRIST_PITCH, 0, 0);
  pwm.setPWM(GRIPPER, 0, 0);
  pwm.setPWM(BASE2, 0, 0);
  pwm.setPWM(SHOULDER2, 0, 0);
  pwm.setPWM(ELBOW2, 0, 0);
}

// Call before/after every servo move. If E-stop has fired, runs the full
// stop sequence and blocks here until the operator clears it, then returns
// true so the caller bails out of whatever move sequence it was mid-way through.
bool checkEStop() {
  if (!eStopActive) return false;
  handleEStop();
  return true;
}

void handleEStop() {
  Serial.println("!!! EMERGENCY STOP TRIGGERED !!! Halting all servos.");
  haltAllServos();
  setStatusColor(255, 0, 0); // red
  lc.clearDisplay(0);
  displayBitmap(XMARK);

  // Alarm + wait for the button to be released
  while (digitalRead(ESTOP_PIN) == HIGH) {
    toneESP32(BUZZER_PIN, 2000, 150);
    delay(300);
  }
  noToneESP32(BUZZER_PIN);

  Serial.println("E-stop released. Press the button again to resume.");
  // Require a deliberate second press before resuming — don't auto-resume
  // just because the button popped back out.
  while (digitalRead(ESTOP_PIN) == LOW) {
    delay(50);
  }
  while (digitalRead(ESTOP_PIN) == HIGH) {
    delay(50); // wait out the confirm press
  }

  Serial.println("Resuming...");
  eStopActive = false;
  lc.clearDisplay(0);

  // Re-home both arms before continuing
  setServoAngle(BASE, 90);      setServoAngle(SHOULDER, 90);   setServoAngle(ELBOW, 90);
  setServoAngle(WRIST_ROT, 0);  setServoAngle(WRIST_PITCH, 90); setServoAngle(GRIPPER, 90);
  setServoAngle(BASE2, 90);     setServoAngle(SHOULDER2, 90);  setServoAngle(ELBOW2, 90);
  delay(500);
}
