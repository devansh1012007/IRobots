#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LedControl.h>   // Install via Library Manager: "LedControl" by Eberhard Fahle

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// =========================================================
// SERVO CHANNEL DEFINITIONS (0-15 on the PCA9685 board)
// ===== ====================================================
// --- ARM 1 (existing) ---
#define BASE        0
#define SHOULDER    1
#define ELBOW       2
#define WRIST_ROT   3
#define WRIST_PITCH 4
#define GRIPPER     5

// --- ARM 2 (NEW second 3-servo arm) ---
// Wire this arm's 3 servos to the next free channels on the SAME PCA9685
// board (these are PCA9685 channel numbers, not Arduino pins).
#define BASE2     6
#define SHOULDER2 7
#define ELBOW2    8

// Pulse Width Limits (Microseconds)
// MG996R: ~600us to 2400us. SG90: ~600us to 2400us.
// We use conservative limits to prevent mechanical binding.
#define SERVOMIN  600
#define SERVOMAX  2400

// =========================================================
// 8x8 DOT MATRIX (MAX7219) — wired per your reference table:
//   MAX7219 VCC -> Arduino 5V     MAX7219 GND -> Arduino GND
//   MAX7219 DIN -> Arduino 11 (MOSI)
//   MAX7219 CS/LOAD -> Arduino 10
//   MAX7219 CLK -> Arduino 13 (SCK)
// =========================================================
#define MATRIX_DIN 11
#define MATRIX_CS  10
#define MATRIX_CLK 13
LedControl lc = LedControl(MATRIX_DIN, MATRIX_CLK, MATRIX_CS, 1); // 1 = one 8x8 module

// =========================================================
// EMERGENCY STOP BUTTON — wired per your breadboard image:
//   Arduino pin 2 (green wire) -> button
//   Button -> 5V (red wire) through the button
//   Resistor pulls pin 2 LOW to GND (black wire) when button is not pressed
// This is an external-pulldown circuit: unpressed = LOW, pressed = HIGH.
// =========================================================
#define ESTOP_PIN 7
volatile bool eStopActive = false;

// =========================================================
// BUZZER — pin not shown in your images, wire a buzzer's + leg here
// (piezo buzzer: pin -> buzzer -> GND, no resistor needed).
// CHANGE THIS if you wire it to a different pin.
// =========================================================
#define BUZZER_PIN 4

// =========================================================
// RGB STATUS LED — pins not shown in your images, wire a common-cathode
// RGB LED here (each color leg through its own ~220 ohm resistor, common
// leg to GND). CHANGE THESE if you wire it differently.
// =========================================================
#define RGB_RED_PIN   3
#define RGB_GREEN_PIN 5
#define RGB_BLUE_PIN  6

// =========================================================
// 8x8 BITMAPS (one byte per row, top row first)
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
  Serial.begin(9600);

  // --- Servo driver ---
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Standard 50Hz for analog servos
  delay(10);

  // --- Dot matrix ---
  lc.shutdown(0, false);   // wake the display up
  lc.setIntensity(0, 8);   // brightness 0-15
  lc.clearDisplay(0);

  // --- RGB status LED ---
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  setStatusColor(0, 0, 0);

  // --- Buzzer ---
  pinMode(BUZZER_PIN, OUTPUT);

  // --- Emergency stop ---
  pinMode(ESTOP_PIN, INPUT); // external pulldown resistor per your wiring image
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), eStopISR, RISING);

  // --- Initialize ARM 1 to neutral ---
  //setServoAngle(BASE, 10);         delay(500);
  setServoAngle(SHOULDER, 100);     delay(500);
  setServoAngle(ELBOW, 97);        delay(500);
  setServoAngle(WRIST_ROT, 100);     delay(500);
  // setServoAngle(WRIST_PITCH, 45);  delay(500);
  setServoAngle(GRIPPER, 90);      delay(500); // Open/Neutral // good enough open // 40- 50 for close 

  // --- Initialize ARM 2 to neutral ---
  setServoAngle(BASE2, 90);        delay(200);
  setServoAngle(SHOULDER2, 90);    delay(200);
  setServoAngle(ELBOW2, 90);       delay(200);
}

void loop() {
  if (checkEStop()) return; // don't start a new cycle mid-stop

  runCountdown();
  if (checkEStop()) return;

  setStatusColor(0, 255, 0); // green: motors running
  Serial.println("Processing...");

  // --- Sweep ARM 1 and AR1M 2 to their first test position ---
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
  if (checkEStop()) return;

  moveJointAnimated(BASE2, 45, 1000);
  if (checkEStop()) return;
  moveJointAnimated(SHOULDER2, 60, 1000);
  if (checkEStop()) return;
  moveJointAnimated(ELBOW2, 50, 1000);
  if (checkEStop()) return;

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

  moveJointAnimated(BASE2, 95, 1000);
  if (checkEStop()) return;
  moveJointAnimated(SHOULDER2, 100, 1000);
  if (checkEStop()) return;
  moveJointAnimated(ELBOW2, 90, 1000);
  if (checkEStop()) return;

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
  tone(BUZZER_PIN, 1000, 150);
  delay(1000);
  if (checkEStop()) return;

  displayBitmap(NUM_2);
  tone(BUZZER_PIN, 1000, 150);
  delay(1000);
  if (checkEStop()) return;

  displayBitmap(NUM_1);
  tone(BUZZER_PIN, 1000, 150);
  delay(1000);
  if (checkEStop()) return;

  tone(BUZZER_PIN, 1800, 300); // longer "go" beep
  lc.clearDisplay(0);
  delay(300);
}

// =========================================================
// SERVO HELPERS
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

void setStatusColor(int r, int g, int b) {
  analogWrite(RGB_RED_PIN, r);
  analogWrite(RGB_GREEN_PIN, g);
  analogWrite(RGB_BLUE_PIN, b);
}

// =========================================================
// EMERGENCY STOP
// =========================================================

// ISR must stay short — just set a flag, do the real work in handleEStop().
void eStopISR() {
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
    tone(BUZZER_PIN, 2000, 150);
    delay(300);
  }
  noTone(BUZZER_PIN);

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
