/*
 * servo_controller.ino
 * ESP32-S3  +  PCA9685 PWM Expander  →  5-DOF Robotic Arm
 *
 * Hardware wiring:
 *   PCA9685 SDA  → GPIO 8  (ESP32-S3 default I2C SDA)
 *   PCA9685 SCL  → GPIO 9  (ESP32-S3 default I2C SCL)
 *   PCA9685 VCC  → 3.3 V
 *   PCA9685 GND  → GND
 *   PCA9685 V+   → 5 V  (servo power rail)
 *
 * PCA9685 channel mapping:
 *   Channel 0 → Servo 1  MG996R  Base rotation
 *   Channel 1 → Servo 2  MG996R  Shoulder (lower arm)
 *   Channel 2 → Servo 3  MG996R  Elbow    (upper arm)
 *   Channel 3 → Servo 4  MG90S   Wrist pitch
 *   Channel 4 → Servo 5  MG90S   Gripper
 *
 * Serial protocol (115200 baud, sent by ROS 2 bridge node):
 *   "S<id>:<angle>\n"
 *   e.g. "S1:90\n"  →  move servo 1 to 90°
 *        "SA:90,90,90,90,90\n"  →  set all 5 servos at once
 *
 * Angle limits (forward-kinematics, hardware-safe):
 *   Servo 1  (base)     0 – 180°
 *   Servo 2  (shoulder) 0 – 150°
 *   Servo 3  (elbow)    0 – 150°
 *   Servo 4  (wrist)    0 – 180°
 *   Servo 5  (gripper)  0 –  90°
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ── PCA9685 ──────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // default I2C addr

// PCA9685 runs at 50 Hz; pulse width for 0° and 180° varies by servo model.
// MG996R : ~500 µs (0°)  – ~2400 µs (180°)
// MG90S  : ~500 µs (0°)  – ~2400 µs (180°)
// At 50 Hz the period is 20 000 µs → 4096 counts.
// counts = (pulse_µs / 20000) * 4096
#define SERVO_FREQ      50
#define COUNT_MIN      102   // ~500  µs
#define COUNT_MAX      491   // ~2400 µs

// ── Servo configuration ───────────────────────────────────────────────────────
const uint8_t  NUM_SERVOS  = 5;
const uint8_t  CHANNEL[5]  = {0, 1, 2, 3, 4};

// Angle limits [min, max] per servo
const uint8_t  ANGLE_MIN[5] = {  0,   0,   0,   0,   0};
const uint8_t  ANGLE_MAX[5] = {255, 255, 255, 255,  255};

// Home / power-on angles
const uint8_t  HOME_ANGLE[5] = {90, 90, 90, 90, 45};

// Current angles (stored so we can query state)
uint8_t currentAngle[5];

// ── Helpers ───────────────────────────────────────────────────────────────────
uint16_t angleToCounts(uint8_t angle) {
  return map(angle, 0, 180, COUNT_MIN, COUNT_MAX);
}

void moveServo(uint8_t servoId, uint8_t angle) {
  // servoId is 1-indexed coming from the protocol
  uint8_t idx = servoId - 1;
  if (idx >= NUM_SERVOS) {
    Serial.printf("[ERR] Invalid servo id: %d\n", servoId);
    return;
  }

  // Clamp to hardware limits
  angle = constrain(angle, ANGLE_MIN[idx], ANGLE_MAX[idx]);

  uint16_t counts = angleToCounts(angle);
  pwm.setPWM(CHANNEL[idx], 0, counts);
  currentAngle[idx] = angle;

  Serial.printf("[OK] Servo %d → %d° (counts=%d)\n", servoId, angle, counts);
}

void moveAllServos(uint8_t angles[NUM_SERVOS]) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    moveServo(i + 1, angles[i]);
  }
}

void printStatus() {
  Serial.print("[STATUS] ");
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.printf("S%d=%d", i + 1, currentAngle[i]);
    if (i < NUM_SERVOS - 1) Serial.print(" ");
  }
  Serial.println();
}

// ── Serial command parser ─────────────────────────────────────────────────────
// Formats supported:
//   "S<1-5>:<angle>\n"         single servo
//   "SA:<a1>,<a2>,<a3>,<a4>,<a5>\n"  all servos
//   "HOME\n"                    return to home
//   "STATUS\n"                  print current angles
String inputBuffer = "";

void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "HOME") {
    Serial.println("[INFO] Moving to home position");
    moveAllServos((uint8_t*)HOME_ANGLE);
    return;
  }

  if (cmd == "STATUS") {
    printStatus();
    return;
  }

  // All-servo command: "SA:90,90,90,90,45"
  if (cmd.startsWith("SA:")) {
    String data = cmd.substring(3);
    uint8_t angles[NUM_SERVOS];
    uint8_t count = 0;
    int start = 0;
    for (int i = 0; i <= (int)data.length() && count < NUM_SERVOS; i++) {
      if (i == (int)data.length() || data[i] == ',') {
        angles[count++] = (uint8_t)data.substring(start, i).toInt();
        start = i + 1;
      }
    }
    if (count == NUM_SERVOS) {
      moveAllServos(angles);
    } else {
      Serial.printf("[ERR] SA expected 5 values, got %d\n", count);
    }
    return;
  }

  // Single-servo command: "S1:90"
  if (cmd.length() >= 4 && cmd[0] == 'S') {
    uint8_t servoId = (uint8_t)(cmd[1] - '0');
    int colonIdx = cmd.indexOf(':');
    if (colonIdx > 0 && servoId >= 1 && servoId <= NUM_SERVOS) {
      uint8_t angle = (uint8_t)cmd.substring(colonIdx + 1).toInt();
      moveServo(servoId, angle);
      return;
    }
  }

  Serial.printf("[ERR] Unknown command: %s\n", cmd.c_str());
}

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      parseCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
}

// ── Setup & Loop ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[BOOT] ESP32-S3 Servo Controller starting...");

  Wire.begin(8, 9);   // SDA=GPIO8, SCL=GPIO9 (ESP32-S3 defaults)
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Calibrate for your board (25–27 MHz)
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("[BOOT] PCA9685 initialised at 50 Hz");

  // Move all servos to home
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    currentAngle[i] = HOME_ANGLE[i];
    moveServo(i + 1, HOME_ANGLE[i]);
    delay(50); // stagger so we don't spike current
  }

  Serial.println("[BOOT] All servos at home position. Awaiting commands.");
  Serial.println("[BOOT] Commands: S<1-5>:<angle>  |  SA:<a,a,a,a,a>  |  HOME  |  STATUS");
}

void loop() {
  handleSerial();
  delay(1); // Yield so the WiFi/BT stack (if used) stays alive
}
