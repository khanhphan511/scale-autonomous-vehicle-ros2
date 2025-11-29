/*
  Arduino MEGA + JZ2407DB-A Dual H-Bridge
  Bluetooth Car Controller — momentary, slower steering + runaway fix
  - L/R are small nudges (±STEP_DEG) instead of snapping to full lock
  - If no steering command arrives for STEER_FREEZE_MS, freeze at current angle
  - C = center; F/B = drive with steering centered
  - G/H/I/J diagonals: drive while nudging left/right once

  Bluetooth on Serial2: TX2=17, RX2=16 (9600 baud)
*/

#include <util/atomic.h>

// ── Pins ──
#define REAR_IN1   24
#define REAR_IN2   25
#define REAR_PWM    5
#define STEER_IN1  26
#define STEER_IN2  27
#define STEER_PWM   6
#define STEER_ENC_A 18
#define STEER_ENC_B 19

// ── Direction signs (flip to -1 if needed) ──
#define DRIVE_SIGN (+1)
#define STEER_SIGN (+1)

// ── Drive tuning ──
const int DRIVE_PWM_FWD  = 30;
const int DRIVE_PWM_REV  = 30;
const int DRIVE_PWM_MAX  = 200;
const int RAMP_STEP_DRIVE = 6; // PWM steps / 10ms

// ── Steering tuning (gentle) ──
const int STEER_PWM_MAX  = 60; // slower than before
const int STEER_PWM_MIN  = 40; // minimum to overcome friction
const int RAMP_STEP_STEER = 2; // slower ramp for smoothness
const int KP_STEER       = 1;  // proportional gain
const int SETTLE_BAND    = 6;  // counts considered on target
const int COUNTS_PER_DEG = 8;  // adjust to your encoder/linkage
const int STEER_MAX_DEG  = 22; // hard limit
//const int STEP_DEG       = 4;  // nudge size per L/R press
const int STEP_DEG       = 7;  // nudge size per L/R press
const int DIAG_STEP_DEG  = 6;  // nudge used by G/H/I/J combos

// ── “Momentary” steering behavior ──
const uint16_t STEER_FREEZE_MS = 250; // after this with no L/R, freeze at current angle

// ── State ──
volatile long steerCount = 0;
long steerTargetCounts   = 0;
int driveTarget = 0;
int driveCmd    = 0;
int steerCmd    = 0;
unsigned long lastSteerCmdMs = 0; // last time an L/R/G/H/I/J/C changed steering

// ── Encoder ISRs ──
void onEncA() {
  uint8_t a = digitalRead(STEER_ENC_A);
  uint8_t b = digitalRead(STEER_ENC_B);
  if ((a ^ b) != 0)
    steerCount++;
  else
    steerCount--;
}

void onEncB() {
  uint8_t a = digitalRead(STEER_ENC_A);
  uint8_t b = digitalRead(STEER_ENC_B);
  if ((a ^ b) == 0)
    steerCount++;
  else
    steerCount--;
}

// ── Motor helpers ──
void rearBrake() {
  digitalWrite(REAR_IN1, LOW);
  digitalWrite(REAR_IN2, LOW);
  analogWrite(REAR_PWM, 255);
}

void steerBrake() {
  analogWrite(STEER_PWM, 0);
  digitalWrite(STEER_IN1, LOW);
  digitalWrite(STEER_IN2, LOW);
}

void rearDriveRaw(int pwm) {
  pwm *= DRIVE_SIGN;
  int p = abs(pwm);
  if (p > 255) p = 255;
  digitalWrite(REAR_IN1, pwm >= 0);
  digitalWrite(REAR_IN2, pwm < 0);
  analogWrite(REAR_PWM, p);
}

void steerDriveRaw(int pwm) {
  pwm *= STEER_SIGN;
  int p = abs(pwm);
  if (p > 255) p = 255;
  digitalWrite(STEER_IN1, pwm >= 0);
  digitalWrite(STEER_IN2, pwm < 0);
  analogWrite(STEER_PWM, p);
}

// ── Ramps ──
void rearRampTo(int target) {
  if (target == 0) {
    driveCmd = 0;
    rearBrake();  // immediate brake
    return;
  }

  if (target > DRIVE_PWM_MAX)  target = DRIVE_PWM_MAX;
  if (target < -DRIVE_PWM_MAX) target = -DRIVE_PWM_MAX;

  if (driveCmd < target)      driveCmd += RAMP_STEP_DRIVE;
  else if (driveCmd > target) driveCmd -= RAMP_STEP_DRIVE;

  rearDriveRaw(driveCmd);
}

void steerRampTo(int target) {
  if (target > STEER_PWM_MAX)  target = STEER_PWM_MAX;
  if (target < -STEER_PWM_MAX) target = -STEER_PWM_MAX;

  if (steerCmd < target)      steerCmd += RAMP_STEP_STEER;
  else if (steerCmd > target) steerCmd -= RAMP_STEP_STEER;

  // ensure minimum while moving
  if (abs(target) > 0 && abs(steerCmd) < STEER_PWM_MIN)
    steerCmd = (target > 0) ? STEER_PWM_MIN : -STEER_PWM_MIN;

  steerDriveRaw(steerCmd);
}

void stopAll() {
  driveTarget = 0;
  steerCmd    = 0;
  driveCmd    = 0;
  rearBrake();
  steerBrake();
}

// ── Steering control ──
long clampDeg(long d) {
  if (d >  STEER_MAX_DEG) d =  STEER_MAX_DEG;
  if (d < -STEER_MAX_DEG) d = -STEER_MAX_DEG;
  return d;
}

long degToCounts(long d) {
  return clampDeg(d) * (long)COUNTS_PER_DEG;
}

void centerSteer() {
  steerTargetCounts = 0;
  lastSteerCmdMs = millis();
}

void addSteerRelativeDeg(long deltaDeg) {
  long currentDegTarget = steerTargetCounts / COUNTS_PER_DEG;
  long nextDeg = clampDeg(currentDegTarget + deltaDeg);
  steerTargetCounts = nextDeg * (long)COUNTS_PER_DEG;
  lastSteerCmdMs = millis();
}

void holdSteering() {
  // “Momentary” behavior: if no steering command for a bit, freeze at current angle
  if (millis() - lastSteerCmdMs > STEER_FREEZE_MS) {
    // set target to NOW so controller stops trying to move
    long snap;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      snap = steerCount;
    }
    steerTargetCounts = snap;
    // Do not update lastSteerCmdMs here (we want to keep freezing until a new command arrives)
  }

  long err = steerTargetCounts - steerCount;
  if (abs(err) <= SETTLE_BAND) {
    steerRampTo(0);
    return;
  }

  long cmd = err * KP_STEER;
  if (cmd >  STEER_PWM_MAX) cmd =  STEER_PWM_MAX;
  if (cmd < -STEER_PWM_MAX) cmd = -STEER_PWM_MAX;

  steerRampTo((int)cmd);
}

// ── Bluetooth parser (Serial2) ──
String rx;

void handleCmd(const String &s) {
  if (s.length() == 0) return;

  char c = toupper(s[0]);

  // Drive only
  if (c == 'F' || c == 'U' || c == '8') {
    driveTarget = +DRIVE_PWM_FWD;
    centerSteer();
    Serial.println(F("[CMD] FWD"));
    return;
  }

  if (c == 'B' || c == 'D' || c == '2') {
    driveTarget = -DRIVE_PWM_REV;
    centerSteer();
    Serial.println(F("[CMD] REV"));
    return;
  }

  if (c == 'S' || c == '5') {
    stopAll();
    Serial.println(F("[CMD] STOP"));
    return;
  }

  // Steering nudges (momentary)
  if (c == 'L' || c == '4') {
    addSteerRelativeDeg(+STEP_DEG);
    Serial.println(F("[CMD] NUDGE LEFT"));
    return;
  }

  if (c == 'R' || c == '6') {
    addSteerRelativeDeg(-STEP_DEG);
    Serial.println(F("[CMD] NUDGE RIGHT"));
    return;
  }

  if (c == 'C') {
    centerSteer();
    Serial.println(F("[CMD] CENTER"));
    return;
  }

  // Diagonal combos (one nudge while commanding drive)
  // (H/I swapped earlier request preserved? not needed for nudges, but we can map as before)
  if (c == 'G') {
    driveTarget = +DRIVE_PWM_FWD;
    addSteerRelativeDeg(+DIAG_STEP_DEG);
    Serial.println(F("[CMD] FWD + nudge LEFT"));
    return;
  }

  if (c == 'H') {
    driveTarget = +DRIVE_PWM_FWD;
    addSteerRelativeDeg(-DIAG_STEP_DEG);
    Serial.println(F("[CMD] FWD + nudge RIGHT"));
    return;
  }

  // flipped mapping kept
  if (c == 'I') {
    driveTarget = -DRIVE_PWM_REV;
    addSteerRelativeDeg(+DIAG_STEP_DEG);
    Serial.println(F("[CMD] REV + nudge LEFT"));
    return;
  }

  // flipped mapping kept
  if (c == 'J') {
    driveTarget = -DRIVE_PWM_REV;
    addSteerRelativeDeg(-DIAG_STEP_DEG);
    Serial.println(F("[CMD] REV + nudge RIGHT"));
    return;
  }

  Serial.print(F("[CMD] Unknown: "));
  Serial.println(s);
}

void readBT() {
  while (Serial2.available()) {
    char ch = (char)Serial2.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rx.trim();
      handleCmd(rx);
      rx = "";
    } else {
      rx += ch;
      if (rx.length() > 8) rx.remove(0);
    }
  }
}

void readUSB() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') continue; // ignore line endings
    char c = toupper(ch);
    String cmd(c); // single-character command: "F", "B", "S", "L", "R", ...
    handleCmd(cmd); // reuse your existing command handler
  }
}

// ── Arduino setup/loop ──
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);

  pinMode(REAR_IN1, OUTPUT);
  pinMode(REAR_IN2, OUTPUT);
  pinMode(REAR_PWM, OUTPUT);

  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);
  pinMode(STEER_PWM, OUTPUT);

  pinMode(STEER_ENC_A, INPUT_PULLUP);
  pinMode(STEER_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(STEER_ENC_A), onEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEER_ENC_B), onEncB, CHANGE);

  stopAll();

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    steerCount = 0;
  }

  centerSteer();

  Serial.println(F("== BT Car Controller (momentary steering) ready =="));
  Serial.println(F("L/R nudges, C centers, F/B drive; G/H/I/J = drive + nudge"));
}

void loop() {
  // readBT();  // for Bluetooth control
  readUSB();   // for USB Serial control (from PC)

  holdSteering();          // close loop & momentary freeze
  rearRampTo(driveTarget); // smooth drive

  delay(10);
}
