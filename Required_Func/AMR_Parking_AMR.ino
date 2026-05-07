#include <Arduino.h>
#include <Servo.h>
#include <math.h>

/*
  AMR_Parking_AMR

  Dedicated parking experiment sketch.

  Goal:
  1. Keep only the reusable hardware layers: motors, front/side ultrasonics, servo, and LDRs.
  2. Use one LDR-based entry assist before parking.
  3. Build parking from scratch using 3 active ultrasonics at a time:
     - GREEN: frontLeft, frontRight, sideRight
     - RED:   frontLeft, frontRight, sideLeft
  4. Be robust when far from target and precise when near target.
  5. Print enough logs to tune burst durations and motor actuation.

  Intended use:
  - Place the robot in the second-stage parking region, already facing the front wall.
  - Use Serial Monitor at 115200.
  - Send:
      g -> run GREEN parking
      r -> run RED parking
      l -> run entry light assist only
      u -> print one ultrasonic snapshot
      p -> print one centered LDR snapshot
      c -> center servo
      s -> stop motors
      h -> help
*/

// ============================================================
// Pin Definitions
// ============================================================
const unsigned long SERIAL_BAUD = 115200UL;

const uint8_t SERVO_PIN = 28;

const uint8_t LDR_LEFT_PIN = A0;
const uint8_t LDR_RIGHT_PIN = A2;

const uint8_t US_L_TRIG = 32;
const uint8_t US_L_ECHO = 33;
const uint8_t US_R_TRIG = 22;
const uint8_t US_R_ECHO = 24;
const uint8_t US_SIDE_L_TRIG = 44;
const uint8_t US_SIDE_L_ECHO = 45;
const uint8_t US_SIDE_R_TRIG = 29;
const uint8_t US_SIDE_R_ECHO = 25;

const uint8_t M1_PWM = 12;
const uint8_t M1_IN1 = 34;
const uint8_t M1_IN2 = 35;

const uint8_t M2_PWM = 8;
const uint8_t M2_IN1 = 37;
const uint8_t M2_IN2 = 36;

const uint8_t M3_PWM = 9;
const uint8_t M3_IN1 = 43;
const uint8_t M3_IN2 = 42;

const uint8_t M4_PWM = 5;
const uint8_t M4_IN1 = A4;
const uint8_t M4_IN2 = A5;

// ============================================================
// Tunables
// ============================================================
const int MOTOR_SIGN[4] = { -1, +1, -1, +1 };
const int MOTOR_TRIM[4] = { 0, 0, 0, 0 };
const int SIDE_BALANCE = 0;

// Minimum actuation force only. Adaptiveness is in burst duration, not PWM.
const int PWM_PARK_FORWARD = 30;
const int PWM_PARK_BACKWARD = 30;
const int PWM_PARK_ROTATE = 29;
const int PWM_PARK_STRAFE = 34;
const int PWM_PARK_STRAFE_KICK = 42;

const unsigned long INVALID_WAIT_MS = 120UL;
const unsigned long ROTATE_SETTLE_MS = 150UL;
const unsigned long DRIVE_SETTLE_MS = 130UL;
const unsigned long STRAFE_SETTLE_MS = 200UL;
const unsigned long FINAL_STABLE_READ_MS = 220UL;
const unsigned long PRECISION_CALM_MS = 260UL;

const float FORWARD_RATIO[4] = { 1.0000f, 0.8863f, 1.0000f, 0.8863f };
const float BACKWARD_RATIO[4] = { 1.0000f, 0.8863f, 1.0000f, 0.8863f };
const float STRAFE_LEFT_RATIO[4] = { 1.0652f, 1.0652f, 0.9348f, 0.9348f };
const float STRAFE_RIGHT_RATIO[4] = { 1.1087f, 1.1087f, 0.8913f, 0.8913f };

// Parking targets from current tuned measurements.
const float GREEN_FRONT_LEFT_CM = 13.4f;
const float GREEN_FRONT_RIGHT_CM = 14.6f;
const float GREEN_SIDE_CM = 22.1f;

const float RED_FRONT_LEFT_CM = 14.4f;
const float RED_FRONT_RIGHT_CM = 15.6f;
const float RED_SIDE_CM = 23.3f;

// Final success tolerances.
const float FINAL_FRONT_LEFT_TOL_CM = 0.40f;
const float FINAL_FRONT_RIGHT_TOL_CM = 0.40f;
const float FINAL_SIDE_TOL_CM = 0.40f;
const float FINAL_DIFF_TOL_CM = 0.35f;

// Stage tolerances.
const float ROUGH_FRONT_DIFF_TOL_CM = 0.80f;
const float ROUGH_SIDE_TOL_CM = 0.80f;
const float ROUGH_DEPTH_TOL_CM = 0.80f;
const int STABLE_COUNT_ROUGH = 2;
const int STABLE_COUNT_FINAL = 5;
const float COARSE_FRONT_SIDE_GATE_DIFF_CM = 0.60f;
const float COARSE_FRONT_SIDE_GATE_AVG_CM = 0.90f;
const float SIDE_DEGRADE_DIFF_LIMIT_CM = 1.40f;
const float SIDE_DEGRADE_AVG_LIMIT_CM = 1.60f;
const float FINAL_SIDE_GATE_DIFF_CM = 0.55f;
const float FINAL_SIDE_GATE_AVG_CM = 0.70f;
const float FINAL_REGIME_NEAR_CM = 0.75f;
const float FINAL_REGIME_FAR_CM = 1.50f;

// Adaptive burst sizing thresholds.
const float ROTATE_FINE_ERR_CM = 0.80f;
const float ROTATE_MED_ERR_CM = 2.00f;
const unsigned long ROTATE_TINY_MS = 24UL;
const unsigned long ROTATE_FINE_MS = 35UL;
const unsigned long ROTATE_MED_MS = 60UL;
const unsigned long ROTATE_COARSE_MS = 95UL;

const float STRAFE_FINE_ERR_CM = 0.80f;
const float STRAFE_MED_ERR_CM = 2.20f;
const unsigned long STRAFE_TINY_MS = 25UL;
const unsigned long STRAFE_FINE_MS = 35UL;
const unsigned long STRAFE_MED_MS = 60UL;
const unsigned long STRAFE_COARSE_MS = 90UL;

const float DRIVE_FINE_ERR_CM = 0.80f;
const float DRIVE_MED_ERR_CM = 2.00f;
const unsigned long DRIVE_TINY_MS = 28UL;
const unsigned long DRIVE_FINE_MS = 40UL;
const unsigned long DRIVE_MED_MS = 60UL;
const unsigned long DRIVE_COARSE_MS = 105UL;

// LDR / light alignment
const int LDR_SAMPLES = 8;
const int LDR_SAMPLE_DELAY_MS = 2;
const float LDR_NORM_MAX = 220.0f;

const int PRESET_DARK_A = 197;
const int PRESET_DARK_B = 188;
const int PRESET_AMBIENT_A = PRESET_DARK_A + 250;
const int PRESET_AMBIENT_B = PRESET_DARK_B + 250;
const float PRESET_GAIN_A = 1.00f;
const float PRESET_GAIN_B = 1.00f;
const float PRESET_LDR_TRIM = 0.0f;

const int SERVO_CENTER = 90;
const int SERVO_MIN = 30;
const int SERVO_MAX = 150;
const int FULL_SCAN_MIN = 55;
const int FULL_SCAN_MAX = 125;
const int FULL_SCAN_STEP_DEG = 5;
const int FULL_SCAN_SETTLE_MS = 120;

const int LIGHT_CENTER_TOL_DEG = 3;
const int MIN_SCAN_SCORE = 20;
const int FULL_SCAN_SCORE_TIE_MARGIN = 3;
const int CENTER_SUM_ACCEPT_MARGIN = 6;
const float CENTER_DIFF_ACCEPT = 3.0f;
const float ENTRY_LDR_WALL_DIST_CM = 9.0f;

// Measured strafe tables at minimum actuation.
const float RIGHT_STRAFE_DISTANCE_CM[] = { 1.05f, 1.45f, 1.83f, 4.60f, 20.8f, 42.8f };
const unsigned long RIGHT_STRAFE_TIME_MS[] = { 100UL, 150UL, 200UL, 500UL, 2000UL, 4000UL };
const float LEFT_STRAFE_DISTANCE_CM[] = { 0.75f, 1.55f, 2.10f, 4.65f, 19.6f, 38.8f };
const unsigned long LEFT_STRAFE_TIME_MS[] = { 100UL, 150UL, 200UL, 500UL, 2000UL, 4000UL };
const int STRAFE_TABLE_COUNT = 6;

// Ultrasonic filtering
const int ULTRA_TIMEOUT_US = 25000;
const int ULTRA_VALID_MIN_CM = 2;
const int ULTRA_VALID_MAX_CM = 400;

// ============================================================
// Data Structures
// ============================================================
struct MotorPins {
  uint8_t pwm;
  uint8_t in1;
  uint8_t in2;
};

struct LdrReading {
  int pan;
  int rawA;
  int rawB;
  int corrA;
  int corrB;
  int sum;
  float normA;
  float normB;
  float diff;
};

struct ScanResult {
  bool found;
  int bestAngle;
  int bestScore;
  LdrReading bestReading;
};

struct UltraPair {
  bool leftValid;
  bool rightValid;
  bool valid;
  float leftCm;
  float rightCm;
  float avgCm;
  float diffCm;
};

struct SidePair {
  bool leftValid;
  bool rightValid;
  float leftCm;
  float rightCm;
};

struct ParkingReadings {
  UltraPair front;
  SidePair side;
};

struct ParkingTarget {
  const char *name;
  bool useRightSide;
  float frontLeftCm;
  float frontRightCm;
  float sideCm;
  float frontLeftTolCm;
  float frontRightTolCm;
  float sideTolCm;
};

// ============================================================
// Globals
// ============================================================
Servo trackerServo;

const MotorPins MOTORS[4] = {
  { M1_PWM, M1_IN1, M1_IN2 },
  { M2_PWM, M2_IN1, M2_IN2 },
  { M3_PWM, M3_IN1, M3_IN2 },
  { M4_PWM, M4_IN1, M4_IN2 }
};

const ParkingTarget GREEN_TARGET = {
  "GREEN", true,
  GREEN_FRONT_LEFT_CM, GREEN_FRONT_RIGHT_CM, GREEN_SIDE_CM,
  FINAL_FRONT_LEFT_TOL_CM, FINAL_FRONT_RIGHT_TOL_CM, FINAL_SIDE_TOL_CM
};

const ParkingTarget RED_TARGET = {
  "RED", false,
  RED_FRONT_LEFT_CM, RED_FRONT_RIGHT_CM, RED_SIDE_CM,
  FINAL_FRONT_LEFT_TOL_CM, FINAL_FRONT_RIGHT_TOL_CM, FINAL_SIDE_TOL_CM
};

int servoPos = SERVO_CENTER;
int darkA = PRESET_DARK_A;
int darkB = PRESET_DARK_B;
int ambientA = PRESET_AMBIENT_A;
int ambientB = PRESET_AMBIENT_B;
float gainA = PRESET_GAIN_A;
float gainB = PRESET_GAIN_B;
float ldrTrim = PRESET_LDR_TRIM;

// ============================================================
// Small Helpers
// ============================================================
float floatAbs(float value) {
  return (value < 0.0f) ? -value : value;
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float median3(float a, float b, float c) {
  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    float t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }
  return b;
}

bool isDistanceValid(float cm) {
  return (cm >= (float)ULTRA_VALID_MIN_CM && cm <= (float)ULTRA_VALID_MAX_CM);
}

unsigned long chooseAdaptiveBurstMs(
  float absError,
  float fineThreshold,
  float mediumThreshold,
  unsigned long fineMs,
  unsigned long mediumMs,
  unsigned long coarseMs
) {
  if (absError <= fineThreshold) return fineMs;
  if (absError <= mediumThreshold) return mediumMs;
  return coarseMs;
}

float targetFrontDiff(const ParkingTarget &target) {
  return target.frontLeftCm - target.frontRightCm;
}

float targetFrontAvg(const ParkingTarget &target) {
  return 0.5f * (target.frontLeftCm + target.frontRightCm);
}

bool isActiveSideValid(const ParkingTarget &target, const SidePair &side) {
  return target.useRightSide ? side.rightValid : side.leftValid;
}

float activeSideCmForTarget(const ParkingTarget &target, const SidePair &side) {
  return target.useRightSide ? side.rightCm : side.leftCm;
}

float maxFloat5(float a, float b, float c, float d, float e) {
  float result = a;
  if (b > result) result = b;
  if (c > result) result = c;
  if (d > result) result = d;
  if (e > result) result = e;
  return result;
}

const char *finalRegimeName(float maxError) {
  if (maxError > FINAL_REGIME_FAR_CM) return "FAR";
  if (maxError > FINAL_REGIME_NEAR_CM) return "NEAR";
  return "PRECISION";
}

unsigned long selectFinalBurstMs(
  float absError,
  float maxError,
  unsigned long tinyMs,
  unsigned long fineMs,
  unsigned long mediumMs,
  unsigned long coarseMs
) {
  if (maxError > FINAL_REGIME_FAR_CM) {
    return chooseAdaptiveBurstMs(absError, 0.80f, 2.00f, fineMs, mediumMs, coarseMs);
  }
  if (maxError > FINAL_REGIME_NEAR_CM) {
    return chooseAdaptiveBurstMs(absError, 0.30f, 0.80f, tinyMs, fineMs, mediumMs);
  }
  return tinyMs;
}

void printActionDelta(const char *phase, const char *action, const ParkingTarget &target, const ParkingReadings &before, const ParkingReadings &after) {
  float beforeSide = activeSideCmForTarget(target, before.side);
  float afterSide = activeSideCmForTarget(target, after.side);

  Serial.print(F("["));
  Serial.print(phase);
  Serial.print(F("_RESULT] action="));
  Serial.print(action);
  Serial.print(F(" beforeDiff="));
  Serial.print(before.front.diffCm, 2);
  Serial.print(F(" afterDiff="));
  Serial.print(after.front.diffCm, 2);
  Serial.print(F(" dDiff="));
  Serial.print(after.front.diffCm - before.front.diffCm, 2);
  Serial.print(F(" beforeAvg="));
  Serial.print(before.front.avgCm, 2);
  Serial.print(F(" afterAvg="));
  Serial.print(after.front.avgCm, 2);
  Serial.print(F(" dAvg="));
  Serial.print(after.front.avgCm - before.front.avgCm, 2);
  Serial.print(F(" beforeSide="));
  Serial.print(beforeSide, 2);
  Serial.print(F(" afterSide="));
  Serial.print(afterSide, 2);
  Serial.print(F(" dSide="));
  Serial.println(afterSide - beforeSide, 2);
}

// ============================================================
// Motor Layer
// ============================================================
void setupMotorPins() {
  pinMode(M1_PWM, OUTPUT); pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT); pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);
  pinMode(M4_PWM, OUTPUT); pinMode(M4_IN1, OUTPUT); pinMode(M4_IN2, OUTPUT);
}

void writeMotorRaw(uint8_t pwmPin, uint8_t in1, uint8_t in2, int signedPwm) {
  signedPwm = constrain(signedPwm, -255, 255);

  if (signedPwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, signedPwm);
  } else if (signedPwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -signedPwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void writeMotorByIndex(uint8_t index, int signedPwm) {
  if (signedPwm != 0) {
    int direction = (signedPwm > 0) ? 1 : -1;
    int magnitude = abs(signedPwm);
    magnitude = constrain(magnitude + MOTOR_TRIM[index], 0, 255);
    signedPwm = direction * magnitude;
  }

  signedPwm *= MOTOR_SIGN[index];
  writeMotorRaw(MOTORS[index].pwm, MOTORS[index].in1, MOTORS[index].in2, signedPwm);
}

void drive4(int pwmM1, int pwmM2, int pwmM3, int pwmM4) {
  writeMotorByIndex(0, pwmM1);
  writeMotorByIndex(1, pwmM2);
  writeMotorByIndex(2, pwmM3);
  writeMotorByIndex(3, pwmM4);
}

int scaleMotionPwm(int basePwm, float ratio) {
  return constrain((int)((float)basePwm * ratio + 0.5f), 0, 255);
}

void driveTank(int leftPwm, int rightPwm) {
  int leftAdjusted = leftPwm - SIDE_BALANCE;
  int rightAdjusted = rightPwm + SIDE_BALANCE;
  drive4(leftAdjusted, rightAdjusted, leftAdjusted, rightAdjusted);
}

void stopMotors() {
  drive4(0, 0, 0, 0);
}

void driveForward(int pwm) {
  drive4(
    scaleMotionPwm(pwm, FORWARD_RATIO[0]),
    scaleMotionPwm(pwm, FORWARD_RATIO[1]),
    scaleMotionPwm(pwm, FORWARD_RATIO[2]),
    scaleMotionPwm(pwm, FORWARD_RATIO[3]));
}

void driveBackward(int pwm) {
  drive4(
    -scaleMotionPwm(pwm, BACKWARD_RATIO[0]),
    -scaleMotionPwm(pwm, BACKWARD_RATIO[1]),
    -scaleMotionPwm(pwm, BACKWARD_RATIO[2]),
    -scaleMotionPwm(pwm, BACKWARD_RATIO[3]));
}

void rotateLeft(int pwm) {
  driveTank(pwm, -pwm);
}

void rotateRight(int pwm) {
  driveTank(-pwm, pwm);
}

void strafeLeft(int pwm) {
  drive4(
    -scaleMotionPwm(pwm, STRAFE_LEFT_RATIO[0]),
    scaleMotionPwm(pwm, STRAFE_LEFT_RATIO[1]),
    scaleMotionPwm(pwm, STRAFE_LEFT_RATIO[2]),
    -scaleMotionPwm(pwm, STRAFE_LEFT_RATIO[3]));
}

void strafeRight(int pwm) {
  drive4(
    scaleMotionPwm(pwm, STRAFE_RIGHT_RATIO[0]),
    -scaleMotionPwm(pwm, STRAFE_RIGHT_RATIO[1]),
    -scaleMotionPwm(pwm, STRAFE_RIGHT_RATIO[2]),
    scaleMotionPwm(pwm, STRAFE_RIGHT_RATIO[3]));
}

void executeTimedRotate(bool rotateLeftNow, unsigned long totalMs) {
  if (rotateLeftNow) {
    rotateLeft(PWM_PARK_ROTATE);
  } else {
    rotateRight(PWM_PARK_ROTATE);
  }
  delay(totalMs);
  stopMotors();
  delay(ROTATE_SETTLE_MS);
}

void executeTimedDrive(bool forwardNow, unsigned long totalMs) {
  if (forwardNow) {
    driveForward(PWM_PARK_FORWARD);
  } else {
    driveBackward(PWM_PARK_BACKWARD);
  }
  delay(totalMs);
  stopMotors();
  delay(DRIVE_SETTLE_MS);
}

void executeTimedStrafe(bool strafeRightNow, unsigned long totalMs) {
  if (totalMs == 0UL) {
    stopMotors();
    return;
  }

  unsigned long kickMs = (totalMs < 90UL) ? totalMs : 90UL;
  unsigned long holdMs = (totalMs > kickMs) ? (totalMs - kickMs) : 0UL;

  if (strafeRightNow) {
    strafeRight(PWM_PARK_STRAFE_KICK);
    delay(kickMs);
    if (holdMs > 0UL) {
      strafeRight(PWM_PARK_STRAFE);
      delay(holdMs);
    }
  } else {
    strafeLeft(PWM_PARK_STRAFE_KICK);
    delay(kickMs);
    if (holdMs > 0UL) {
      strafeLeft(PWM_PARK_STRAFE);
      delay(holdMs);
    }
  }

  stopMotors();
  delay(STRAFE_SETTLE_MS);
}

// ============================================================
// Servo / LDR Layer
// ============================================================
void writeServoAngle(int angle) {
  servoPos = constrain(angle, SERVO_MIN, SERVO_MAX);
  trackerServo.write(servoPos);
}

void centerServo() {
  writeServoAngle(SERVO_CENTER);
}

int readAverage(int pin, int samples = LDR_SAMPLES) {
  long sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += analogRead(pin);
    delay(LDR_SAMPLE_DELAY_MS);
  }
  return (int)(sum / samples);
}

float normalizeLdr(int rawValue, int darkValue, int ambientValue) {
  int span = ambientValue - darkValue;
  if (span < 1) span = 1;

  float normalized = 100.0f * (float)(rawValue - darkValue) / (float)span;
  return clampFloat(normalized, 0.0f, LDR_NORM_MAX);
}

LdrReading readLdrAtAngle(int angle) {
  LdrReading reading;

  writeServoAngle(angle);
  delay(FULL_SCAN_SETTLE_MS);

  reading.pan = servoPos;
  reading.rawA = readAverage(LDR_LEFT_PIN);
  reading.rawB = readAverage(LDR_RIGHT_PIN);

  reading.corrA = reading.rawA - darkA;
  reading.corrB = reading.rawB - darkB;
  if (reading.corrA < 0) reading.corrA = 0;
  if (reading.corrB < 0) reading.corrB = 0;

  reading.normA = clampFloat(normalizeLdr(reading.rawA, darkA, ambientA) * gainA + ldrTrim, 0.0f, LDR_NORM_MAX);
  reading.normB = clampFloat(normalizeLdr(reading.rawB, darkB, ambientB) * gainB - ldrTrim, 0.0f, LDR_NORM_MAX);
  reading.diff = reading.normA - reading.normB;
  reading.sum = reading.corrA + reading.corrB;
  return reading;
}

void printLdrReading(const __FlashStringHelper *label, const LdrReading &reading) {
  Serial.print(F("["));
  Serial.print(label);
  Serial.print(F("] pan="));
  Serial.print(reading.pan);
  Serial.print(F(" rawA="));
  Serial.print(reading.rawA);
  Serial.print(F(" rawB="));
  Serial.print(reading.rawB);
  Serial.print(F(" corrA="));
  Serial.print(reading.corrA);
  Serial.print(F(" corrB="));
  Serial.print(reading.corrB);
  Serial.print(F(" normA="));
  Serial.print(reading.normA, 1);
  Serial.print(F(" normB="));
  Serial.print(reading.normB, 1);
  Serial.print(F(" diff="));
  Serial.print(reading.diff, 1);
  Serial.print(F(" sum="));
  Serial.println(reading.sum);
}

void printScanResult(const __FlashStringHelper *label, const ScanResult &scan) {
  Serial.print(F("["));
  Serial.print(label);
  Serial.print(F("] found="));
  Serial.print(scan.found ? F("true") : F("false"));
  Serial.print(F(" bestAngle="));
  Serial.print(scan.bestAngle);
  Serial.print(F(" bestScore="));
  Serial.print(scan.bestScore);
  Serial.print(F(" bestDiff="));
  Serial.print(scan.bestReading.diff, 1);
  Serial.print(F(" angleError="));
  Serial.println(scan.bestAngle - SERVO_CENTER);
}

ScanResult runScanAndLockRange(int scanMin, int scanMax, int scanStepDeg, int tieMargin, const __FlashStringHelper *sampleLabel) {
  ScanResult result;
  result.found = false;
  result.bestAngle = SERVO_CENTER;
  result.bestScore = -1;

  const int maxSamples = ((SERVO_MAX - SERVO_MIN) / 2) + 8;
  LdrReading readings[maxSamples];
  int readingCount = 0;
  int globalBestScore = -1;

  Serial.print(F("[SCAN] range="));
  Serial.print(scanMin);
  Serial.print(F(".."));
  Serial.print(scanMax);
  Serial.print(F(" step="));
  Serial.println(scanStepDeg);

  for (int angle = scanMin; angle <= scanMax; angle += scanStepDeg) {
    LdrReading reading = readLdrAtAngle(angle);
    printLdrReading(sampleLabel, reading);
    if (readingCount < maxSamples) {
      readings[readingCount++] = reading;
    }
    if (reading.sum > globalBestScore) {
      globalBestScore = reading.sum;
    }
  }

  if (readingCount == 0 || globalBestScore < MIN_SCAN_SCORE) {
    centerServo();
    return result;
  }

  for (int i = 0; i < readingCount; ++i) {
    const LdrReading &reading = readings[i];
    if (globalBestScore - reading.sum > tieMargin) continue;

    bool better = false;
    if (!result.found) {
      better = true;
    } else {
      int currentOffset = abs(reading.pan - SERVO_CENTER);
      int bestOffset = abs(result.bestAngle - SERVO_CENTER);
      float currentDiffAbs = floatAbs(reading.diff);
      float bestDiffAbs = floatAbs(result.bestReading.diff);

      if (currentOffset < bestOffset) {
        better = true;
      } else if (currentOffset == bestOffset && currentDiffAbs < bestDiffAbs - 0.5f) {
        better = true;
      } else if (currentOffset == bestOffset && currentDiffAbs <= bestDiffAbs + 0.5f && reading.sum > result.bestScore) {
        better = true;
      }
    }

    if (better) {
      result.found = true;
      result.bestAngle = reading.pan;
      result.bestScore = reading.sum;
      result.bestReading = reading;
    }
  }

  if (!result.found) {
    centerServo();
    return result;
  }

  writeServoAngle(result.bestAngle);
  delay(FULL_SCAN_SETTLE_MS);
  result.bestReading = readLdrAtAngle(result.bestAngle);
  return result;
}

float computeLateralShiftCm(int angleErrorDeg, float sensorWallDistCm) {
  int angleAbs = abs(angleErrorDeg);
  if (angleAbs <= 0 || sensorWallDistCm <= 0.0f) return 0.0f;
  float angleRad = (float)angleAbs * 3.14159265f / 180.0f;
  return sensorWallDistCm * tan(angleRad);
}

unsigned long interpolateStrafeMsFromDistance(float distanceCm, const float *distanceTableCm, const unsigned long *timeTableMs) {
  if (distanceCm <= 0.0f) return 0UL;

  if (distanceCm <= distanceTableCm[0]) {
    return (unsigned long)((distanceCm * (float)timeTableMs[0] / distanceTableCm[0]) + 0.5f);
  }

  for (int i = 1; i < STRAFE_TABLE_COUNT; ++i) {
    if (distanceCm <= distanceTableCm[i]) {
      float spanCm = distanceTableCm[i] - distanceTableCm[i - 1];
      float ratio = (distanceCm - distanceTableCm[i - 1]) / spanCm;
      float timeMs = (float)timeTableMs[i - 1] + ratio * (float)(timeTableMs[i] - timeTableMs[i - 1]);
      return (unsigned long)(timeMs + 0.5f);
    }
  }

  return timeTableMs[STRAFE_TABLE_COUNT - 1];
}

unsigned long computeRightStrafeMsForDistanceCm(float distanceCm) {
  return interpolateStrafeMsFromDistance(distanceCm, RIGHT_STRAFE_DISTANCE_CM, RIGHT_STRAFE_TIME_MS);
}

unsigned long computeLeftStrafeMsForDistanceCm(float distanceCm) {
  return interpolateStrafeMsFromDistance(distanceCm, LEFT_STRAFE_DISTANCE_CM, LEFT_STRAFE_TIME_MS);
}

bool runEntryLightAssist() {
  Serial.println(F("[ENTRY_LDR] Start entry light assist."));
  centerServo();
  stopMotors();
  delay(FULL_SCAN_SETTLE_MS);

  ScanResult scan = runScanAndLockRange(FULL_SCAN_MIN, FULL_SCAN_MAX, FULL_SCAN_STEP_DEG, FULL_SCAN_SCORE_TIE_MARGIN, F("ENTRY_SCAN"));
  printScanResult(F("ENTRY_SCAN_RESULT"), scan);

  if (!scan.found) {
    Serial.println(F("[ENTRY_LDR] No valid peak. Skip LDR assist."));
    centerServo();
    return true;
  }

  int angleError = scan.bestAngle - SERVO_CENTER;
  LdrReading centerReading = readLdrAtAngle(SERVO_CENTER);
  int centerGap = scan.bestScore - centerReading.sum;
  bool plateauCentered =
    (centerGap <= CENTER_SUM_ACCEPT_MARGIN) &&
    (floatAbs(centerReading.diff) <= CENTER_DIFF_ACCEPT);

  Serial.print(F("[ENTRY_LDR] angleError="));
  Serial.print(angleError);
  Serial.print(F(" centerGap="));
  Serial.print(centerGap);
  Serial.print(F(" centerDiff="));
  Serial.print(centerReading.diff, 1);
  Serial.print(F(" plateauCentered="));
  Serial.println(plateauCentered ? F("true") : F("false"));

  if (abs(angleError) <= LIGHT_CENTER_TOL_DEG || plateauCentered) {
    centerServo();
    Serial.println(F("[ENTRY_LDR] Already acceptable."));
    return true;
  }

  bool strafeRightNow = (angleError < 0);
  float lateralShiftCm = computeLateralShiftCm(angleError, ENTRY_LDR_WALL_DIST_CM);
  unsigned long shiftMs = strafeRightNow
    ? computeRightStrafeMsForDistanceCm(lateralShiftCm)
    : computeLeftStrafeMsForDistanceCm(lateralShiftCm);

  Serial.print(F("[ENTRY_LDR] dir="));
  Serial.print(strafeRightNow ? F("RIGHT") : F("LEFT"));
  Serial.print(F(" lateralShiftCm="));
  Serial.print(lateralShiftCm, 2);
  Serial.print(F(" shiftMs="));
  Serial.println(shiftMs);

  executeTimedStrafe(strafeRightNow, shiftMs);
  centerServo();
  return true;
}

// ============================================================
// Ultrasonic Layer
// ============================================================
float readDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ULTRA_TIMEOUT_US);
  if (duration == 0UL) return 999.0f;

  float distance = duration * 0.0343f * 0.5f;
  if (!isDistanceValid(distance)) return 999.0f;
  return distance;
}

UltraPair readFrontPair() {
  UltraPair pair;

  float left1 = readDistanceCm(US_L_TRIG, US_L_ECHO);
  delay(8);
  float left2 = readDistanceCm(US_L_TRIG, US_L_ECHO);
  delay(8);
  float left3 = readDistanceCm(US_L_TRIG, US_L_ECHO);
  delay(12);

  float right1 = readDistanceCm(US_R_TRIG, US_R_ECHO);
  delay(8);
  float right2 = readDistanceCm(US_R_TRIG, US_R_ECHO);
  delay(8);
  float right3 = readDistanceCm(US_R_TRIG, US_R_ECHO);

  pair.leftCm = median3(left1, left2, left3);
  pair.rightCm = median3(right1, right2, right3);
  pair.leftValid = isDistanceValid(pair.leftCm);
  pair.rightValid = isDistanceValid(pair.rightCm);
  pair.valid = pair.leftValid && pair.rightValid;

  if (pair.valid) {
    pair.avgCm = 0.5f * (pair.leftCm + pair.rightCm);
    pair.diffCm = pair.leftCm - pair.rightCm;
  } else if (pair.leftValid) {
    pair.avgCm = pair.leftCm;
    pair.diffCm = 0.0f;
  } else if (pair.rightValid) {
    pair.avgCm = pair.rightCm;
    pair.diffCm = 0.0f;
  } else {
    pair.avgCm = 999.0f;
    pair.diffCm = 0.0f;
  }

  return pair;
}

SidePair readSidePair() {
  SidePair pair;

  float left1 = readDistanceCm(US_SIDE_L_TRIG, US_SIDE_L_ECHO);
  delay(8);
  float left2 = readDistanceCm(US_SIDE_L_TRIG, US_SIDE_L_ECHO);
  delay(8);
  float left3 = readDistanceCm(US_SIDE_L_TRIG, US_SIDE_L_ECHO);
  delay(12);

  float right1 = readDistanceCm(US_SIDE_R_TRIG, US_SIDE_R_ECHO);
  delay(8);
  float right2 = readDistanceCm(US_SIDE_R_TRIG, US_SIDE_R_ECHO);
  delay(8);
  float right3 = readDistanceCm(US_SIDE_R_TRIG, US_SIDE_R_ECHO);

  pair.leftCm = median3(left1, left2, left3);
  pair.rightCm = median3(right1, right2, right3);
  pair.leftValid = isDistanceValid(pair.leftCm);
  pair.rightValid = isDistanceValid(pair.rightCm);
  return pair;
}

ParkingReadings readParkingReadings() {
  ParkingReadings readings;
  readings.front = readFrontPair();
  delay(12);
  readings.side = readSidePair();
  return readings;
}

void printFrontPair(const __FlashStringHelper *label, const UltraPair &pair) {
  Serial.print(F("["));
  Serial.print(label);
  Serial.print(F("] fl="));
  Serial.print(pair.leftCm, 1);
  Serial.print(F(" fr="));
  Serial.print(pair.rightCm, 1);
  Serial.print(F(" avg="));
  Serial.print(pair.avgCm, 1);
  Serial.print(F(" diff="));
  Serial.print(pair.diffCm, 2);
  Serial.print(F(" valid="));
  Serial.println(pair.valid ? F("true") : F("false"));
}

void printParkingReadings(const __FlashStringHelper *label, const ParkingReadings &readings) {
  Serial.print(F("["));
  Serial.print(label);
  Serial.print(F("] frontLeft="));
  Serial.print(readings.front.leftCm, 1);
  Serial.print(F(" frontRight="));
  Serial.print(readings.front.rightCm, 1);
  Serial.print(F(" sideLeft="));
  Serial.print(readings.side.leftCm, 1);
  Serial.print(F(" sideRight="));
  Serial.println(readings.side.rightCm, 1);
}

// ============================================================
// Parking Controller
// ============================================================
bool runFrontGeometryStage(const ParkingTarget &target) {
  const float targetDiff = targetFrontDiff(target);
  const float targetAvg = targetFrontAvg(target);
  int stableCount = 0;

  Serial.print(F("[PARK_FRONT] targetDiff="));
  Serial.print(targetDiff, 2);
  Serial.print(F(" targetAvg="));
  Serial.println(targetAvg, 2);

  while (true) {
    UltraPair pair = readFrontPair();
    printFrontPair(F("PARK_FRONT"), pair);

    if (!pair.valid) {
      stableCount = 0;
      stopMotors();
      delay(INVALID_WAIT_MS);
      continue;
    }

    float diffError = pair.diffCm - targetDiff;
    float avgError = pair.avgCm - targetAvg;
    Serial.print(F("[PARK_FRONT] diffError="));
    Serial.print(diffError, 2);
    Serial.print(F(" avgError="));
    Serial.println(avgError, 2);

    if (floatAbs(diffError) <= ROUGH_FRONT_DIFF_TOL_CM &&
        floatAbs(avgError) <= ROUGH_DEPTH_TOL_CM) {
      stableCount++;
      Serial.print(F("[PARK_FRONT] stableCount="));
      Serial.println(stableCount);
      if (stableCount >= STABLE_COUNT_ROUGH) {
        stopMotors();
        Serial.println(F("[PARK_FRONT] Front geometry+depth locked."));
        return true;
      }
      delay(INVALID_WAIT_MS);
      continue;
    }

    stableCount = 0;

    if (floatAbs(diffError) > ROUGH_FRONT_DIFF_TOL_CM) {
      unsigned long burstMs = chooseAdaptiveBurstMs(
        floatAbs(diffError),
        ROTATE_FINE_ERR_CM,
        ROTATE_MED_ERR_CM,
        ROTATE_FINE_MS,
        ROTATE_MED_MS,
        ROTATE_COARSE_MS
      );

      bool rotateLeftNow = (diffError > 0.0f);
      Serial.print(F("[PARK_FRONT_CMD] action="));
      Serial.print(rotateLeftNow ? F("ROTATE_LEFT") : F("ROTATE_RIGHT"));
      Serial.print(F(" burstMs="));
      Serial.println(burstMs);
      executeTimedRotate(rotateLeftNow, burstMs);
      continue;
    }

    unsigned long burstMs = chooseAdaptiveBurstMs(
      floatAbs(avgError),
      DRIVE_FINE_ERR_CM,
      DRIVE_MED_ERR_CM,
      DRIVE_FINE_MS,
      DRIVE_MED_MS,
      DRIVE_COARSE_MS
    );

    bool driveForwardNow = (avgError > 0.0f);
    Serial.print(F("[PARK_FRONT_CMD] action="));
    Serial.print(driveForwardNow ? F("FORWARD") : F("BACKWARD"));
    Serial.print(F(" burstMs="));
    Serial.println(burstMs);
    executeTimedDrive(driveForwardNow, burstMs);
  }
}

bool runSideDistanceStage(const ParkingTarget &target) {
  int stableCount = 0;

  Serial.print(F("[PARK_SIDE] targetSide="));
  Serial.print(target.sideCm, 2);
  Serial.print(F(" sensor="));
  Serial.println(target.useRightSide ? F("RIGHT") : F("LEFT"));

  while (true) {
    SidePair pair = readSidePair();
    bool activeSideValid = target.useRightSide ? pair.rightValid : pair.leftValid;
    float activeSideCm = target.useRightSide ? pair.rightCm : pair.leftCm;

    Serial.print(F("[PARK_SIDE] left="));
    Serial.print(pair.leftCm, 1);
    Serial.print(F(" right="));
    Serial.print(pair.rightCm, 1);
    Serial.print(F(" active="));
    Serial.println(activeSideCm, 1);

    if (!activeSideValid) {
      stableCount = 0;
      stopMotors();
      delay(INVALID_WAIT_MS);
      continue;
    }

    float sideError = activeSideCm - target.sideCm;
    Serial.print(F("[PARK_SIDE] sideError="));
    Serial.println(sideError, 2);

    if (floatAbs(sideError) <= ROUGH_SIDE_TOL_CM) {
      stableCount++;
      Serial.print(F("[PARK_SIDE] stableCount="));
      Serial.println(stableCount);
      if (stableCount >= STABLE_COUNT_ROUGH) {
        stopMotors();
        Serial.println(F("[PARK_SIDE] Side distance locked."));
        return true;
      }
      delay(INVALID_WAIT_MS);
      continue;
    }

    stableCount = 0;

    unsigned long burstMs = chooseAdaptiveBurstMs(
      floatAbs(sideError),
      STRAFE_FINE_ERR_CM,
      STRAFE_MED_ERR_CM,
      STRAFE_FINE_MS,
      STRAFE_MED_MS,
      STRAFE_COARSE_MS
    );

    bool strafeTowardTarget = sideError > 0.0f ? target.useRightSide : !target.useRightSide;
    Serial.print(F("[PARK_SIDE_CMD] action="));
    Serial.print(strafeTowardTarget ? F("STRAFE_RIGHT") : F("STRAFE_LEFT"));
    Serial.print(F(" burstMs="));
    Serial.println(burstMs);
    executeTimedStrafe(strafeTowardTarget, burstMs);
  }
}

bool runCoarseInterleavedStage(const ParkingTarget &target) {
  const float targetDiff = targetFrontDiff(target);
  const float targetAvg = targetFrontAvg(target);
  int stableCount = 0;

  Serial.print(F("[COARSE] targetAvg="));
  Serial.print(targetAvg, 2);
  Serial.print(F(" targetDiff="));
  Serial.print(targetDiff, 2);
  Serial.print(F(" targetSide="));
  Serial.println(target.sideCm, 2);

  while (true) {
    ParkingReadings before = readParkingReadings();
    printParkingReadings(F("COARSE"), before);

    if (!before.front.leftValid || !before.front.rightValid) {
      stableCount = 0;
      stopMotors();
      delay(INVALID_WAIT_MS);
      continue;
    }

    if (!isActiveSideValid(target, before.side)) {
      stableCount = 0;
      stopMotors();
      delay(INVALID_WAIT_MS);
      continue;
    }

    float sideCm = activeSideCmForTarget(target, before.side);
    float diffError = before.front.diffCm - targetDiff;
    float avgError = before.front.avgCm - targetAvg;
    float sideError = sideCm - target.sideCm;

    bool frontOk = (floatAbs(diffError) <= ROUGH_FRONT_DIFF_TOL_CM) &&
                   (floatAbs(avgError) <= ROUGH_DEPTH_TOL_CM);
    bool sideOk = floatAbs(sideError) <= ROUGH_SIDE_TOL_CM;
    bool frontGateOk = (floatAbs(diffError) <= COARSE_FRONT_SIDE_GATE_DIFF_CM) &&
                       (floatAbs(avgError) <= COARSE_FRONT_SIDE_GATE_AVG_CM);

    Serial.print(F("[COARSE_ERR] eDiff="));
    Serial.print(diffError, 2);
    Serial.print(F(" eAvg="));
    Serial.print(avgError, 2);
    Serial.print(F(" eSide="));
    Serial.print(sideError, 2);
    Serial.print(F(" frontGate="));
    Serial.println(frontGateOk ? F("true") : F("false"));

    if (frontOk && sideOk) {
      stableCount++;
      Serial.print(F("[COARSE] stableCount="));
      Serial.println(stableCount);
      if (stableCount >= STABLE_COUNT_ROUGH) {
        stopMotors();
        Serial.println(F("[COARSE] Coarse lock complete."));
        return true;
      }
      delay(INVALID_WAIT_MS);
      continue;
    }

    stableCount = 0;

    if (!frontGateOk) {
      if (floatAbs(diffError) > ROUGH_FRONT_DIFF_TOL_CM) {
        unsigned long burstMs = chooseAdaptiveBurstMs(
          floatAbs(diffError),
          ROTATE_FINE_ERR_CM,
          ROTATE_MED_ERR_CM,
          ROTATE_FINE_MS,
          ROTATE_MED_MS,
          ROTATE_COARSE_MS
        );
        bool rotateLeftNow = (diffError > 0.0f);
        Serial.print(F("[COARSE_FRONT_CMD] action="));
        Serial.print(rotateLeftNow ? F("ROTATE_LEFT") : F("ROTATE_RIGHT"));
        Serial.print(F(" burstMs="));
        Serial.println(burstMs);
        executeTimedRotate(rotateLeftNow, burstMs);
        ParkingReadings after = readParkingReadings();
        printActionDelta("COARSE_FRONT", rotateLeftNow ? "ROTATE_LEFT" : "ROTATE_RIGHT", target, before, after);
        continue;
      }

      unsigned long burstMs = chooseAdaptiveBurstMs(
        floatAbs(avgError),
        DRIVE_FINE_ERR_CM,
        DRIVE_MED_ERR_CM,
        DRIVE_FINE_MS,
        DRIVE_MED_MS,
        DRIVE_COARSE_MS
      );
      bool driveForwardNow = (avgError > 0.0f);
      Serial.print(F("[COARSE_FRONT_CMD] action="));
      Serial.print(driveForwardNow ? F("FORWARD") : F("BACKWARD"));
      Serial.print(F(" burstMs="));
      Serial.println(burstMs);
      executeTimedDrive(driveForwardNow, burstMs);
      ParkingReadings after = readParkingReadings();
      printActionDelta("COARSE_FRONT", driveForwardNow ? "FORWARD" : "BACKWARD", target, before, after);
      continue;
    }

    if (!sideOk) {
      unsigned long burstMs = chooseAdaptiveBurstMs(
        floatAbs(sideError),
        STRAFE_FINE_ERR_CM,
        STRAFE_MED_ERR_CM,
        STRAFE_FINE_MS,
        STRAFE_MED_MS,
        STRAFE_COARSE_MS
      );
      bool strafeTowardTarget = sideError > 0.0f ? target.useRightSide : !target.useRightSide;
      Serial.print(F("[COARSE_SIDE_CMD] action="));
      Serial.print(strafeTowardTarget ? F("STRAFE_RIGHT") : F("STRAFE_LEFT"));
      Serial.print(F(" burstMs="));
      Serial.println(burstMs);
      executeTimedStrafe(strafeTowardTarget, burstMs);
      ParkingReadings after = readParkingReadings();
      printActionDelta("COARSE_SIDE", strafeTowardTarget ? "STRAFE_RIGHT" : "STRAFE_LEFT", target, before, after);

      if (after.front.valid) {
        float afterDiffError = after.front.diffCm - targetDiff;
        float afterAvgError = after.front.avgCm - targetAvg;
        if (floatAbs(afterDiffError) > SIDE_DEGRADE_DIFF_LIMIT_CM ||
            floatAbs(afterAvgError) > SIDE_DEGRADE_AVG_LIMIT_CM) {
          Serial.print(F("[COARSE_SIDE_RESULT] frontDegraded=true afterDiffError="));
          Serial.print(afterDiffError, 2);
          Serial.print(F(" afterAvgError="));
          Serial.println(afterAvgError, 2);
        }
      }
      continue;
    }

    if (!frontOk) {
      unsigned long burstMs = chooseAdaptiveBurstMs(
        floatAbs(avgError),
        DRIVE_FINE_ERR_CM,
        DRIVE_MED_ERR_CM,
        DRIVE_FINE_MS,
        DRIVE_MED_MS,
        DRIVE_COARSE_MS
      );
      bool driveForwardNow = (avgError > 0.0f);
      Serial.print(F("[COARSE_FRONT_CMD] action="));
      Serial.print(driveForwardNow ? F("FORWARD") : F("BACKWARD"));
      Serial.print(F(" burstMs="));
      Serial.println(burstMs);
      executeTimedDrive(driveForwardNow, burstMs);
      ParkingReadings after = readParkingReadings();
      printActionDelta("COARSE_FRONT", driveForwardNow ? "FORWARD" : "BACKWARD", target, before, after);
      continue;
    }
  }
}

bool runFinalTrim(const ParkingTarget &target) {
  const float targetDiff = targetFrontDiff(target);
  const float targetAvg = targetFrontAvg(target);
  int stableCount = 0;

  Serial.print(F("[PARK_FINAL] targetAvg="));
  Serial.print(targetAvg, 2);
  Serial.print(F(" targetDiff="));
  Serial.println(targetDiff, 2);

  while (true) {
    ParkingReadings before = readParkingReadings();
    printParkingReadings(F("PARK_FINAL"), before);

    if (!before.front.leftValid || !before.front.rightValid) {
      stableCount = 0;
      stopMotors();
      delay(INVALID_WAIT_MS);
      continue;
    }

    bool activeSideValid = isActiveSideValid(target, before.side);
    float activeSideCm = activeSideCmForTarget(target, before.side);
    if (!activeSideValid) {
      stableCount = 0;
      stopMotors();
      delay(INVALID_WAIT_MS);
      continue;
    }

    float errorFL = before.front.leftCm - target.frontLeftCm;
    float errorFR = before.front.rightCm - target.frontRightCm;
    float sideError = activeSideCm - target.sideCm;
    float diffError = before.front.diffCm - targetDiff;
    float avgError = before.front.avgCm - targetAvg;

    bool frontLeftOk = floatAbs(errorFL) <= target.frontLeftTolCm;
    bool frontRightOk = floatAbs(errorFR) <= target.frontRightTolCm;
    bool sideOk = floatAbs(sideError) <= target.sideTolCm;
    bool diffOk = floatAbs(diffError) <= FINAL_DIFF_TOL_CM;
    bool avgOk = floatAbs(avgError) <= FINAL_SIDE_GATE_AVG_CM;
    bool frontGateOk = (floatAbs(diffError) <= FINAL_SIDE_GATE_DIFF_CM) && avgOk;
    float maxError = maxFloat5(floatAbs(errorFL), floatAbs(errorFR), floatAbs(sideError), floatAbs(diffError), floatAbs(avgError));

    Serial.print(F("[PARK_FINAL_ERR] eFL="));
    Serial.print(errorFL, 2);
    Serial.print(F(" eFR="));
    Serial.print(errorFR, 2);
    Serial.print(F(" eSide="));
    Serial.print(sideError, 2);
    Serial.print(F(" eDiff="));
    Serial.print(diffError, 2);
    Serial.print(F(" eAvg="));
    Serial.print(avgError, 2);
    Serial.print(F(" regime="));
    Serial.print(finalRegimeName(maxError));
    Serial.print(F(" frontGate="));
    Serial.println(frontGateOk ? F("true") : F("false"));

    if (frontLeftOk && frontRightOk && sideOk && diffOk && avgOk) {
      stableCount++;
      Serial.print(F("[PARK_FINAL] withinTolerance stableCount="));
      Serial.println(stableCount);
      if (stableCount >= STABLE_COUNT_FINAL) {
        stopMotors();
        Serial.println(F("[PARK_DONE] Parking aligned."));
        return true;
      }
      delay(FINAL_STABLE_READ_MS);
      continue;
    }

    stableCount = 0;

    if (maxError <= FINAL_REGIME_NEAR_CM) {
      Serial.println(F("[PARK_FINAL] calm reread before precision move."));
      stopMotors();
      delay(PRECISION_CALM_MS);
      before = readParkingReadings();
      printParkingReadings(F("PARK_FINAL_CALM"), before);
      if (!before.front.leftValid || !before.front.rightValid || !isActiveSideValid(target, before.side)) {
        continue;
      }

      activeSideCm = activeSideCmForTarget(target, before.side);
      errorFL = before.front.leftCm - target.frontLeftCm;
      errorFR = before.front.rightCm - target.frontRightCm;
      sideError = activeSideCm - target.sideCm;
      diffError = before.front.diffCm - targetDiff;
      avgError = before.front.avgCm - targetAvg;
      frontLeftOk = floatAbs(errorFL) <= target.frontLeftTolCm;
      frontRightOk = floatAbs(errorFR) <= target.frontRightTolCm;
      sideOk = floatAbs(sideError) <= target.sideTolCm;
      diffOk = floatAbs(diffError) <= FINAL_DIFF_TOL_CM;
      avgOk = floatAbs(avgError) <= FINAL_SIDE_GATE_AVG_CM;
      frontGateOk = (floatAbs(diffError) <= FINAL_SIDE_GATE_DIFF_CM) && avgOk;
      maxError = maxFloat5(floatAbs(errorFL), floatAbs(errorFR), floatAbs(sideError), floatAbs(diffError), floatAbs(avgError));
    }

    if (floatAbs(diffError) > FINAL_DIFF_TOL_CM) {
      unsigned long burstMs = selectFinalBurstMs(
        floatAbs(diffError),
        maxError,
        ROTATE_TINY_MS,
        ROTATE_FINE_MS,
        ROTATE_MED_MS,
        ROTATE_COARSE_MS
      );
      bool rotateLeftNow = (diffError > 0.0f);
      Serial.print(F("[PARK_FINAL_CMD] priority=DIFF action="));
      Serial.print(rotateLeftNow ? F("ROTATE_LEFT") : F("ROTATE_RIGHT"));
      Serial.print(F(" burstMs="));
      Serial.println(burstMs);
      executeTimedRotate(rotateLeftNow, burstMs);
      ParkingReadings after = readParkingReadings();
      printActionDelta("FINAL_DIFF", rotateLeftNow ? "ROTATE_LEFT" : "ROTATE_RIGHT", target, before, after);
      continue;
    }

    if (floatAbs(sideError) > target.sideTolCm && frontGateOk) {
      unsigned long burstMs = selectFinalBurstMs(
        floatAbs(sideError),
        maxError,
        STRAFE_TINY_MS,
        STRAFE_FINE_MS,
        STRAFE_MED_MS,
        STRAFE_COARSE_MS
      );
      bool strafeTowardTarget = sideError > 0.0f ? target.useRightSide : !target.useRightSide;
      Serial.print(F("[PARK_FINAL_CMD] priority=SIDE action="));
      Serial.print(strafeTowardTarget ? F("STRAFE_RIGHT") : F("STRAFE_LEFT"));
      Serial.print(F(" burstMs="));
      Serial.println(burstMs);
      executeTimedStrafe(strafeTowardTarget, burstMs);
      ParkingReadings after = readParkingReadings();
      printActionDelta("FINAL_SIDE", strafeTowardTarget ? "STRAFE_RIGHT" : "STRAFE_LEFT", target, before, after);
      if (after.front.valid) {
        float afterDiffError = after.front.diffCm - targetDiff;
        float afterAvgError = after.front.avgCm - targetAvg;
        if (floatAbs(afterDiffError) > SIDE_DEGRADE_DIFF_LIMIT_CM ||
            floatAbs(afterAvgError) > SIDE_DEGRADE_AVG_LIMIT_CM) {
          Serial.print(F("[FINAL_SIDE_RESULT] frontDegraded=true afterDiffError="));
          Serial.print(afterDiffError, 2);
          Serial.print(F(" afterAvgError="));
          Serial.println(afterAvgError, 2);
        }
      }
      continue;
    }

    if (floatAbs(avgError) > 0.25f || !frontLeftOk || !frontRightOk) {
      unsigned long burstMs = selectFinalBurstMs(
        floatAbs(avgError),
        maxError,
        DRIVE_TINY_MS,
        DRIVE_FINE_MS,
        DRIVE_MED_MS,
        DRIVE_COARSE_MS
      );
      bool driveForwardNow = (avgError > 0.0f);
      Serial.print(F("[PARK_FINAL_CMD] priority=DEPTH action="));
      Serial.print(driveForwardNow ? F("FORWARD") : F("BACKWARD"));
      Serial.print(F(" burstMs="));
      Serial.println(burstMs);
      executeTimedDrive(driveForwardNow, burstMs);
      ParkingReadings after = readParkingReadings();
      printActionDelta("FINAL_DEPTH", driveForwardNow ? "FORWARD" : "BACKWARD", target, before, after);
      continue;
    }

    if (!frontGateOk) {
      bool rotateLeftNow = (diffError > 0.0f);
      unsigned long burstMs = ROTATE_TINY_MS;
      Serial.print(F("[PARK_FINAL_CMD] priority=RELOCK action="));
      Serial.print(rotateLeftNow ? F("ROTATE_LEFT") : F("ROTATE_RIGHT"));
      Serial.print(F(" burstMs="));
      Serial.println(burstMs);
      executeTimedRotate(rotateLeftNow, burstMs);
      ParkingReadings after = readParkingReadings();
      printActionDelta("FINAL_RELOCK", rotateLeftNow ? "ROTATE_LEFT" : "ROTATE_RIGHT", target, before, after);
      continue;
    }

    if (!sideOk) {
      bool strafeTowardTarget = sideError > 0.0f ? target.useRightSide : !target.useRightSide;
      Serial.print(F("[PARK_FINAL_CMD] priority=SIDE_FINE action="));
      Serial.print(strafeTowardTarget ? F("STRAFE_RIGHT") : F("STRAFE_LEFT"));
      Serial.print(F(" burstMs="));
      Serial.println(STRAFE_TINY_MS);
      executeTimedStrafe(strafeTowardTarget, STRAFE_TINY_MS);
      ParkingReadings after = readParkingReadings();
      printActionDelta("FINAL_SIDE_FINE", strafeTowardTarget ? "STRAFE_RIGHT" : "STRAFE_LEFT", target, before, after);
    } else if (!diffOk) {
      bool rotateLeftNow = (diffError > 0.0f);
      Serial.print(F("[PARK_FINAL_CMD] priority=DIFF_FINE action="));
      Serial.print(rotateLeftNow ? F("ROTATE_LEFT") : F("ROTATE_RIGHT"));
      Serial.print(F(" burstMs="));
      Serial.println(ROTATE_TINY_MS);
      executeTimedRotate(rotateLeftNow, ROTATE_TINY_MS);
      ParkingReadings after = readParkingReadings();
      printActionDelta("FINAL_DIFF_FINE", rotateLeftNow ? "ROTATE_LEFT" : "ROTATE_RIGHT", target, before, after);
    } else {
      bool driveForwardNow = (avgError > 0.0f);
      Serial.print(F("[PARK_FINAL_CMD] priority=DEPTH_FINE action="));
      Serial.print(driveForwardNow ? F("FORWARD") : F("BACKWARD"));
      Serial.print(F(" burstMs="));
      Serial.println(DRIVE_TINY_MS);
      executeTimedDrive(driveForwardNow, DRIVE_TINY_MS);
      ParkingReadings after = readParkingReadings();
      printActionDelta("FINAL_DEPTH_FINE", driveForwardNow ? "FORWARD" : "BACKWARD", target, before, after);
    }
  }
}

bool runParkingSequence(const ParkingTarget &target) {
  Serial.println();
  Serial.println(F("===== PARK START ====="));
  Serial.print(F("[PARK_TARGET] zone="));
  Serial.print(target.name);
  Serial.print(F(" targetFL="));
  Serial.print(target.frontLeftCm, 2);
  Serial.print(F(" targetFR="));
  Serial.print(target.frontRightCm, 2);
  Serial.print(F(" targetSide="));
  Serial.print(target.sideCm, 2);
  Serial.print(F(" useSide="));
  Serial.println(target.useRightSide ? F("RIGHT") : F("LEFT"));

  centerServo();
  stopMotors();
  delay(INVALID_WAIT_MS);

  runEntryLightAssist();
  runCoarseInterleavedStage(target);
  return runFinalTrim(target);
}

// ============================================================
// Serial UI
// ============================================================
void printHelp() {
  Serial.println();
  Serial.println(F("=== AMR Parking AMR Commands ==="));
  Serial.println(F("g : run GREEN parking"));
  Serial.println(F("r : run RED parking"));
  Serial.println(F("l : run entry light assist only"));
  Serial.println(F("u : print one ultrasonic snapshot"));
  Serial.println(F("p : print one centered LDR snapshot"));
  Serial.println(F("c : center servo"));
  Serial.println(F("s : stop motors"));
  Serial.println(F("h or ? : help"));
  Serial.println();
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') continue;
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');

    switch (c) {
      case 'G':
        runParkingSequence(GREEN_TARGET);
        break;

      case 'R':
        runParkingSequence(RED_TARGET);
        break;

      case 'L':
        runEntryLightAssist();
        break;

      case 'U':
        {
          ParkingReadings readings = readParkingReadings();
          printParkingReadings(F("LIVE"), readings);
          break;
        }

      case 'P':
        {
          LdrReading reading = readLdrAtAngle(SERVO_CENTER);
          printLdrReading(F("NOW"), reading);
          break;
        }

      case 'C':
        centerServo();
        Serial.println(F("[CMD] Servo centered."));
        break;

      case 'S':
        stopMotors();
        Serial.println(F("[CMD] STOP"));
        break;

      case 'H':
      case '?':
        printHelp();
        break;

      default:
        Serial.print(F("Unknown command: "));
        Serial.println(c);
        Serial.println(F("Press H for help."));
        break;
    }
  }
}

// ============================================================
// Setup / Loop
// ============================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  setupMotorPins();
  stopMotors();

  pinMode(US_L_TRIG, OUTPUT);
  pinMode(US_L_ECHO, INPUT);
  pinMode(US_R_TRIG, OUTPUT);
  pinMode(US_R_ECHO, INPUT);
  pinMode(US_SIDE_L_TRIG, OUTPUT);
  pinMode(US_SIDE_L_ECHO, INPUT);
  pinMode(US_SIDE_R_TRIG, OUTPUT);
  pinMode(US_SIDE_R_ECHO, INPUT);
  digitalWrite(US_L_TRIG, LOW);
  digitalWrite(US_R_TRIG, LOW);
  digitalWrite(US_SIDE_L_TRIG, LOW);
  digitalWrite(US_SIDE_R_TRIG, LOW);

  pinMode(LDR_LEFT_PIN, INPUT);
  pinMode(LDR_RIGHT_PIN, INPUT);

  trackerServo.attach(SERVO_PIN);
  centerServo();

  Serial.println(F("AMR parking experiment sketch"));
  Serial.println(F("Use G/R to run parking experiments."));
  Serial.println(F("This sketch is parking-only: one LDR entry assist, then staged ultrasonic parking."));
  printHelp();
}

void loop() {
  handleSerial();
}
