#include <Arduino.h>
#include <Servo.h>

/*
Stage goal:
1. Move forward until either ultrasonic first sees the wall at about 35 cm.
2. Rotate until the body is perpendicular to the wall using the two ultrasonic sensors.
3. Move until average wall distance is about 33 cm.
4. Use one wide scan to find which side the light is on.
5. Re-center the servo and strafe toward that side by a small bounded amount based on the scan angle.
6. Re-check perpendicular + 33 cm + light angle with a narrower rescan and repeat if needed.
8. Stop when the light is centered.

Reference-only source:
- pin assignments
- scan timing
- ultrasonic tuning starting points
- PWM starting points
- forward trim values
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

const uint8_t COLOR_S0_PIN = 48;
const uint8_t COLOR_S1_PIN = 47;
const uint8_t COLOR_S2_PIN = 40;
const uint8_t COLOR_S3_PIN = 41;
const uint8_t COLOR_OUT_PIN = 30;

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
const int LDR_SAMPLES = 8;
const int DARK_CAL_SAMPLES = 80;
const int LDR_SAMPLE_DELAY_MS = 2;
const float LDR_NORM_MAX = 220.0f;

const bool USE_PRESET_LDR_CALIBRATION = true;
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
const int NARROW_SCAN_MIN = 72;
const int NARROW_SCAN_MAX = 108;
const int NARROW_SCAN_STEP_DEG = 2;
const int FULL_SCAN_SETTLE_MS = 120;

const int MOTOR_SIGN[4] = { -1, +1, -1, +1 };
const int MOTOR_TRIM[4] = { 0, 0, 0, 0 };
const int SIDE_BALANCE = 0;

const int PWM_INITIAL_FORWARD = 60;
const int PWM_APPROACH_FAST = 42;
const int PWM_APPROACH_SLOW = 30;
const int PWM_BACKUP = 34;
const int PWM_STRAFE = 34;
const int PWM_STRAFE_KICK = 42;
const int PWM_ALIGN_BASE = 24;
const int PWM_ALIGN_EXTRA = 12;
const int PWM_TURN_90 = 28;

const unsigned long INITIAL_FORWARD_TIMEOUT_MS = 12000UL;
const unsigned long INITIAL_FORWARD_LOOP_DELAY_MS = 25UL;
const unsigned long ALIGN_LOOP_DELAY_MS = 20UL;
const unsigned long APPROACH_BURST_MS = 140UL;
const unsigned long BACKUP_BURST_MS = 100UL;
const unsigned long STAGE1_MIN_STRAFE_MS = 150UL;
const unsigned long STAGE1_MAX_STEP_SHIFT_MS = 4200UL;
const unsigned long STAGE2_MIN_STRAFE_MS = 100UL;
const unsigned long STAGE2_MAX_STEP_SHIFT_MS = 1000UL;
const unsigned long STRAFE_KICK_MS = 90UL;
const unsigned long STRAFE_SETTLE_MS = 150UL;
const unsigned long FLOOR_BACKUP_MS = 180UL;
const unsigned long TURN_LEFT_90_MS = 2400UL;
const unsigned long TURN_RIGHT_90_MS = 2550UL;

const float TARGET_WALL_DIST_CM = 33.0f;
const float INITIAL_FORWARD_DETECT_CM = 35.0f;
const float TARGET_DEPTH_TOL_CM = 1.5f;
const float STAGE1_LDR_WALL_DIST_CM = 24.5f;
const float SECOND_STAGE_LDR_WALL_DIST_CM = 9.0f;
const float SECOND_STAGE_TARGET_WALL_DIST_CM = 17.0f;
const float SECOND_STAGE_TARGET_DEPTH_TOL_CM = 1.0f;
const float PERP_TOL_CM = 1.5f;
const float STOP_DIST_CM = 24.0f;
const float SECOND_STAGE_STOP_DIST_CM = 12.0f;
const float SLOW_DIST_CM = 38.0f;

const int CENTER_TOL_DEG = 3;
const int FINAL_CENTER_TOL_DEG = 6;
const int STEP_SHIFT_SMALL_ANGLE_DEG = 12;
const int STEP_SHIFT_MAX_ANGLE_DEG = 45;
const int MIN_SCAN_SCORE = 20;
const int FULL_SCAN_SCORE_TIE_MARGIN = 3;
const int NARROW_SCAN_SCORE_TIE_MARGIN = 1;
const int CENTER_SUM_ACCEPT_MARGIN = 6;
const float CENTER_DIFF_ACCEPT = 3.0f;

const int ULTRA_TIMEOUT_US = 25000;
const int ULTRA_VALID_MIN_CM = 2;
const int ULTRA_VALID_MAX_CM = 400;
const int ULTRA_RETRIES = 6;
const int ALIGN_RETRIES = 40;
const int APPROACH_RETRIES = 40;
const int ALIGN_STABLE_COUNT = 2;
const int COLOR_NUM_SAMPLES = 10;
const int COLOR_SETTLE_MS = 5;
const unsigned long COLOR_TIMEOUT_US = 30000UL;

const float FORWARD_RATIO[4] = { 1.0000f, 0.8462f, 1.0000f, 0.8462f };
const float BACKWARD_RATIO[4] = { 1.0000f, 0.8462f, 1.0000f, 0.8462f };
const float STRAFE_LEFT_RATIO[4] = { 1.0652f, 1.0652f, 0.9348f, 0.9348f };
const float STRAFE_RIGHT_RATIO[4] = { 1.1087f, 1.1087f, 0.8913f, 0.8913f };

// Positive scan angle error means the peak is at a higher pan angle.
// On this robot, that must command a left strafe, not a right strafe.
const bool POSITIVE_ANGLE_MEANS_STRAFE_RIGHT = false;

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

enum ColorDecision {
  COLOR_ERROR,
  COLOR_FLOOR,
  COLOR_RED,
  COLOR_GREEN
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

int servoPos = SERVO_CENTER;
int darkA = 0;
int darkB = 0;
int ambientA = 0;
int ambientB = 0;
float gainA = PRESET_GAIN_A;
float gainB = PRESET_GAIN_B;
float ldrTrim = PRESET_LDR_TRIM;
unsigned long lastColorR = 0;
unsigned long lastColorG = 0;
unsigned long lastColorB = 0;
float lastColorRgRatio = 0.0f;
ColorDecision lastColorDecision = COLOR_ERROR;

bool autoHasRun = false;

// ============================================================
// Forward Declarations
// ============================================================
void setupMotorPins();
void stopMotors();
void centerServo();
void writeServoAngle(int angle);
int readAverage(int pin, int samples = LDR_SAMPLES);
float normalizeLdr(int rawValue, int darkValue, int ambientValue);
void loadPresetLdrCalibration();
void doDarkCalibration();
LdrReading readLdrAtAngle(int angle);
ScanResult runScanAndLockRange(int scanMin, int scanMax, int scanStepDeg, int tieMargin, const __FlashStringHelper *sampleLabel);
ScanResult runFullScanAndLock();
void writeMotorRaw(uint8_t pwmPin, uint8_t in1, uint8_t in2, int signedPwm);
void writeMotorByIndex(uint8_t index, int signedPwm);
void drive4(int pwmM1, int pwmM2, int pwmM3, int pwmM4);
void driveTank(int leftPwm, int rightPwm);
void driveForward(int pwm);
void driveBackward(int pwm);
void rotateLeft(int pwm);
void rotateRight(int pwm);
void strafeLeft(int pwm);
void strafeRight(int pwm);
float readDistanceCm(uint8_t trigPin, uint8_t echoPin);
UltraPair readUltrasonicPair();
bool alignPerpendicular();
bool initialForwardUntilWallSeen();
bool moveToTargetDistance();
bool alignPerpendicularWithStop(float stopGuardCm, const __FlashStringHelper *label);
bool moveToTargetDistanceWithStop(float targetDistanceCm, float targetTolCm, float stopGuardCm, const __FlashStringHelper *label);
bool advanceUntilAnySensorReads(float detectCm, float stopGuardCm, const __FlashStringHelper *label);
unsigned long readColorChannelAvg(bool s2State, bool s3State);
ColorDecision classifyColor(unsigned long rRaw, unsigned long gRaw, unsigned long bRaw);
const char *colorName(ColorDecision color);
ColorDecision readAndClassifyColor();
void printColorReading();
bool backupFromColorFloor();
void rotateNinetyDegrees(bool clockwise);
bool handlePostStageColor();
LdrReading readLdrNow();
void executeTimedStrafe(bool strafeRightNow, unsigned long totalMs);
unsigned long computeParallelStepMs(int angleErrorDeg, unsigned long minStrafeMs, unsigned long maxStrafeMs);
float computeStage1LateralShiftCm(int angleErrorDeg, float sensorWallDistCm);
unsigned long computeRightStrafeMsForDistanceCm(float distanceCm);
unsigned long computeLeftStrafeMsForDistanceCm(float distanceCm);
bool runParallelStepShiftByDistance(bool strafeRightNow, int angleErrorDeg, float sensorWallDistCm, const __FlashStringHelper *label);
bool runParallelStepShift(bool strafeRightNow, int angleErrorDeg, unsigned long minStrafeMs, unsigned long maxStrafeMs, const __FlashStringHelper *label);
bool centerLightBySumPeak();
bool centerLightBySumPeakAtDistance(float targetDistanceCm, float targetTolCm, float stopGuardCm, bool useDistanceModel, float sensorWallDistCm, unsigned long minStrafeMs, unsigned long maxStrafeMs);
bool runAutoSequence();
void printHelp();
void printUltraPair(const __FlashStringHelper *label, const UltraPair &pair);
void printLdrReading(const __FlashStringHelper *label, const LdrReading &reading);
void printScanResult(const __FlashStringHelper *label, const ScanResult &scan);
float floatAbs(float value);
float clampFloat(float value, float minValue, float maxValue);
float median3(float a, float b, float c);
bool isDistanceValid(float cm);

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

// ============================================================
// Motor Layer
// ============================================================
void setupMotorPins() {
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M4_PWM, OUTPUT);
  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
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

int readAverage(int pin, int samples) {
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

void loadPresetLdrCalibration() {
  darkA = PRESET_DARK_A;
  darkB = PRESET_DARK_B;
  ambientA = PRESET_AMBIENT_A;
  ambientB = PRESET_AMBIENT_B;
  gainA = PRESET_GAIN_A;
  gainB = PRESET_GAIN_B;
  ldrTrim = PRESET_LDR_TRIM;

  Serial.print(F("[CAL] preset darkA="));
  Serial.print(darkA);
  Serial.print(F(" darkB="));
  Serial.print(darkB);
  Serial.print(F(" ambientA="));
  Serial.print(ambientA);
  Serial.print(F(" ambientB="));
  Serial.print(ambientB);
  Serial.print(F(" gainA="));
  Serial.print(gainA, 2);
  Serial.print(F(" gainB="));
  Serial.print(gainB, 2);
  Serial.print(F(" trim="));
  Serial.println(ldrTrim, 1);
}

void doDarkCalibration() {
  stopMotors();
  centerServo();

  Serial.println(F("[DARK] Cover BOTH photoresistors now."));
  Serial.println(F("[DARK] Calibration starts in 5 seconds..."));
  delay(5000);

  long sumA = 0;
  long sumB = 0;
  for (int i = 0; i < DARK_CAL_SAMPLES; ++i) {
    sumA += analogRead(LDR_LEFT_PIN);
    sumB += analogRead(LDR_RIGHT_PIN);
    delay(5);
  }

  darkA = (int)(sumA / DARK_CAL_SAMPLES);
  darkB = (int)(sumB / DARK_CAL_SAMPLES);
  ambientA = darkA + 250;
  ambientB = darkB + 250;
  gainA = 1.0f;
  gainB = 1.0f;
  ldrTrim = 0.0f;

  Serial.print(F("[DARK] darkA="));
  Serial.print(darkA);
  Serial.print(F(" darkB="));
  Serial.print(darkB);
  Serial.print(F(" ambientA="));
  Serial.print(ambientA);
  Serial.print(F(" ambientB="));
  Serial.println(ambientB);

  Serial.println(F("[DARK] Remove your hand."));
  delay(1500);
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

ScanResult runFullScanAndLock() {
  return runScanAndLockRange(FULL_SCAN_MIN, FULL_SCAN_MAX, FULL_SCAN_STEP_DEG, FULL_SCAN_SCORE_TIE_MARGIN, F("SCAN"));
}

ScanResult runScanAndLockRange(int scanMin, int scanMax, int scanStepDeg, int tieMargin, const __FlashStringHelper *sampleLabel) {
  ScanResult result;
  result.found = false;
  result.bestAngle = SERVO_CENTER;
  result.bestScore = -1;

  Serial.println(F("[SCAN] ===== Start scan ====="));
  Serial.print(F("[SCAN] range="));
  Serial.print(scanMin);
  Serial.print(F(".."));
  Serial.print(scanMax);
  Serial.print(F(" step="));
  Serial.println(scanStepDeg);

  const int maxSamples = ((SERVO_MAX - SERVO_MIN) / NARROW_SCAN_STEP_DEG) + 8;
  LdrReading readings[maxSamples];
  int readingCount = 0;
  int globalBestScore = -1;

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
    Serial.println(F("[SCAN] No valid light peak found."));
    centerServo();
    result.found = false;
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
    Serial.println(F("[SCAN] No shortlist candidate found."));
    centerServo();
    return result;
  }

  writeServoAngle(result.bestAngle);
  delay(FULL_SCAN_SETTLE_MS);
  result.bestReading = readLdrAtAngle(result.bestAngle);

  Serial.print(F("[SCAN] Best angle="));
  Serial.print(result.bestAngle);
  Serial.print(F(" bestScore="));
  Serial.println(result.bestScore);

  return result;
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

UltraPair readUltrasonicPair() {
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

// ============================================================
// Color Sensor Layer
// ============================================================
unsigned long readColorChannelAvg(bool s2State, bool s3State) {
  digitalWrite(COLOR_S2_PIN, s2State);
  digitalWrite(COLOR_S3_PIN, s3State);
  delay(COLOR_SETTLE_MS);

  unsigned long sum = 0UL;
  int validCount = 0;

  for (int i = 0; i < COLOR_NUM_SAMPLES; ++i) {
    unsigned long t = pulseIn(COLOR_OUT_PIN, LOW, COLOR_TIMEOUT_US);
    if (t > 0UL) {
      sum += t;
      validCount++;
    }
  }

  if (validCount == 0) return 0UL;
  return sum / (unsigned long)validCount;
}

ColorDecision classifyColor(unsigned long rRaw, unsigned long gRaw, unsigned long bRaw) {
  if (rRaw == 0UL || gRaw == 0UL || bRaw == 0UL) {
    return COLOR_ERROR;
  }

  float rgRatio = (float)gRaw / (float)rRaw;

  if (rgRatio < 0.90f) {
    return COLOR_GREEN;
  } else if (rgRatio > 2.50f) {
    return COLOR_RED;
  }

  return COLOR_FLOOR;
}

const char *colorName(ColorDecision color) {
  switch (color) {
    case COLOR_RED: return "RED";
    case COLOR_GREEN: return "GREEN";
    case COLOR_FLOOR: return "FLOOR";
    default: return "ERROR";
  }
}

ColorDecision readAndClassifyColor() {
  lastColorR = readColorChannelAvg(LOW, LOW);
  lastColorG = readColorChannelAvg(HIGH, HIGH);
  lastColorB = readColorChannelAvg(LOW, HIGH);

  lastColorRgRatio = 0.0f;
  if (lastColorR != 0UL) {
    lastColorRgRatio = (float)lastColorG / (float)lastColorR;
  }

  lastColorDecision = classifyColor(lastColorR, lastColorG, lastColorB);
  return lastColorDecision;
}

void printColorReading() {
  Serial.print(F("[COLOR] r="));
  Serial.print(lastColorR);
  Serial.print(F(" g="));
  Serial.print(lastColorG);
  Serial.print(F(" b="));
  Serial.print(lastColorB);
  Serial.print(F(" rg="));
  Serial.print(lastColorRgRatio, 3);
  Serial.print(F(" decision="));
  Serial.println(colorName(lastColorDecision));
}

bool backupFromColorFloor() {
  Serial.println(F("[COLOR] FLOOR/ERROR -> backup 2 cm"));
  driveBackward(PWM_BACKUP);
  delay(FLOOR_BACKUP_MS);
  stopMotors();
  delay(STRAFE_SETTLE_MS);
  return true;
}

void rotateNinetyDegrees(bool clockwise) {
  Serial.print(F("[TURN] 90 deg "));
  Serial.println(clockwise ? F("CLOCKWISE") : F("ANTI_CLOCKWISE"));

  if (clockwise) {
    Serial.print(F("[TURN] pwm="));
    Serial.print(PWM_TURN_90);
    Serial.print(F(" durationMs="));
    Serial.println(TURN_RIGHT_90_MS);
    rotateRight(PWM_TURN_90);
    delay(TURN_RIGHT_90_MS);
  } else {
    Serial.print(F("[TURN] pwm="));
    Serial.print(PWM_TURN_90);
    Serial.print(F(" durationMs="));
    Serial.println(TURN_LEFT_90_MS);
    rotateLeft(PWM_TURN_90);
    delay(TURN_LEFT_90_MS);
  }
  stopMotors();
  delay(STRAFE_SETTLE_MS);
}

bool handlePostStageColor() {
  centerServo();
  stopMotors();
  delay(FULL_SCAN_SETTLE_MS);

  ColorDecision color = readAndClassifyColor();
  printColorReading();

  if (color == COLOR_FLOOR || color == COLOR_ERROR) {
    return backupFromColorFloor();
  }

  rotateNinetyDegrees(color == COLOR_GREEN);

  if (!alignPerpendicularWithStop(SECOND_STAGE_STOP_DIST_CM, F("SIDE_ALIGN"))) return false;
  if (!advanceUntilAnySensorReads(SECOND_STAGE_TARGET_WALL_DIST_CM, SECOND_STAGE_STOP_DIST_CM, F("SIDE_APPROACH"))) return false;
  if (!centerLightBySumPeakAtDistance(SECOND_STAGE_TARGET_WALL_DIST_CM, SECOND_STAGE_TARGET_DEPTH_TOL_CM, SECOND_STAGE_STOP_DIST_CM, true, SECOND_STAGE_LDR_WALL_DIST_CM, STAGE2_MIN_STRAFE_MS, STAGE2_MAX_STEP_SHIFT_MS)) return false;

  Serial.println(F("[SECOND] Side-wall target reached and centered."));
  return true;
}

// ============================================================
// Control Phases
// ============================================================
bool initialForwardUntilWallSeen() {
  Serial.println(F("[INITIAL] Start forward-until-first-35cm detection"));

  unsigned long startMs = millis();
  driveForward(PWM_INITIAL_FORWARD);

  while ((millis() - startMs) <= INITIAL_FORWARD_TIMEOUT_MS) {
    UltraPair pair = readUltrasonicPair();
    printUltraPair(F("INITIAL"), pair);

    bool leftHit = pair.leftValid && pair.leftCm <= INITIAL_FORWARD_DETECT_CM;
    bool rightHit = pair.rightValid && pair.rightCm <= INITIAL_FORWARD_DETECT_CM;

    if ((pair.leftValid && pair.leftCm <= STOP_DIST_CM) || (pair.rightValid && pair.rightCm <= STOP_DIST_CM)) {
      stopMotors();
      Serial.println(F("[INITIAL] Stop-distance guard hit."));
      return false;
    }

    if (leftHit || rightHit) {
      stopMotors();
      Serial.println(F("[INITIAL] First wall detection reached."));
      delay(STRAFE_SETTLE_MS);
      return true;
    }

    driveForward(PWM_INITIAL_FORWARD);
    delay(INITIAL_FORWARD_LOOP_DELAY_MS);
  }

  stopMotors();
  Serial.println(F("[INITIAL] Timed out before either ultrasonic saw 35 cm."));
  return false;
}

bool alignPerpendicular() {
  return alignPerpendicularWithStop(STOP_DIST_CM, F("ALIGN"));
}

bool alignPerpendicularWithStop(float stopGuardCm, const __FlashStringHelper *label) {
  Serial.print(F("["));
  Serial.print(label);
  Serial.println(F("] Start perpendicular alignment"));
  int stableCount = 0;
  int lastTurnDir = 0;
  int lastPwm = -1;

  for (int attempt = 0; attempt < ALIGN_RETRIES; ++attempt) {
    UltraPair pair = readUltrasonicPair();
    printUltraPair(label, pair);

    if (!pair.valid) {
      stableCount = 0;

      if ((pair.leftValid && pair.leftCm <= stopGuardCm) || (pair.rightValid && pair.rightCm <= stopGuardCm)) {
        stopMotors();
        Serial.print(F("["));
        Serial.print(label);
        Serial.println(F("] One valid ultrasonic is too close to wall, abort."));
        return false;
      }

      if (pair.leftValid && !pair.rightValid) {
        int pwm = PWM_ALIGN_BASE + PWM_ALIGN_EXTRA;
        if (lastTurnDir != -1 || lastPwm != pwm) {
          Serial.print(F("["));
          Serial.print(label);
          Serial.print(F("] right missing, rotate LEFT pwm="));
          Serial.println(pwm);
          lastTurnDir = -1;
          lastPwm = pwm;
        }
        rotateLeft(pwm);
        delay(ALIGN_LOOP_DELAY_MS);
        continue;
      }

      if (pair.rightValid && !pair.leftValid) {
        int pwm = PWM_ALIGN_BASE + PWM_ALIGN_EXTRA;
        if (lastTurnDir != 1 || lastPwm != pwm) {
          Serial.print(F("["));
          Serial.print(label);
          Serial.print(F("] left missing, rotate RIGHT pwm="));
          Serial.println(pwm);
          lastTurnDir = 1;
          lastPwm = pwm;
        }
        rotateRight(pwm);
        delay(ALIGN_LOOP_DELAY_MS);
        continue;
      }

      stopMotors();
      lastTurnDir = 0;
      lastPwm = -1;
      delay(STRAFE_SETTLE_MS);
      continue;
    }

    if (pair.avgCm <= stopGuardCm) {
      stopMotors();
      Serial.print(F("["));
      Serial.print(label);
      Serial.println(F("] Too close to wall, abort."));
      return false;
    }

    if (floatAbs(pair.diffCm) <= PERP_TOL_CM) {
      stableCount++;
      if (stableCount >= ALIGN_STABLE_COUNT) {
        stopMotors();
        Serial.print(F("["));
        Serial.print(label);
        Serial.println(F("] Perpendicular OK"));
        return true;
      }

      delay(ALIGN_LOOP_DELAY_MS);
      continue;
    }

    stableCount = 0;
    float diffAbs = floatAbs(pair.diffCm);
    int pwm = PWM_ALIGN_BASE;

    if (diffAbs >= 8.0f) {
      pwm += PWM_ALIGN_EXTRA;
    } else if (diffAbs >= 5.0f) {
      pwm += PWM_ALIGN_EXTRA;
    } else if (diffAbs >= 3.0f) {
      pwm += PWM_ALIGN_EXTRA / 2;
    }

    int turnDir = (pair.diffCm > 0.0f) ? -1 : 1;
    if (turnDir != lastTurnDir || pwm != lastPwm) {
      Serial.print(F("["));
      Serial.print(label);
      Serial.print(F("] turnPwm="));
      Serial.print(pwm);
      Serial.print(F(" turnDir="));
      Serial.println(turnDir > 0 ? F("RIGHT") : F("LEFT"));
      lastTurnDir = turnDir;
      lastPwm = pwm;
    }

    if (turnDir < 0) {
      rotateLeft(pwm);
    } else {
      rotateRight(pwm);
    }

    delay(ALIGN_LOOP_DELAY_MS);
  }

  stopMotors();
  Serial.print(F("["));
  Serial.print(label);
  Serial.println(F("] Failed to converge."));
  return false;
}

bool moveToTargetDistance() {
  return moveToTargetDistanceWithStop(TARGET_WALL_DIST_CM, TARGET_DEPTH_TOL_CM, STOP_DIST_CM, F("DEPTH"));
}

bool moveToTargetDistanceWithStop(float targetDistanceCm, float targetTolCm, float stopGuardCm, const __FlashStringHelper *label) {
  Serial.print(F("["));
  Serial.print(label);
  Serial.print(F("] Start approach to "));
  Serial.print(targetDistanceCm, 1);
  Serial.println(F(" cm"));

  for (int attempt = 0; attempt < APPROACH_RETRIES; ++attempt) {
    UltraPair pair = readUltrasonicPair();
    printUltraPair(label, pair);

    if (!pair.valid) {
      stopMotors();
      delay(STRAFE_SETTLE_MS);
      continue;
    }

    if (floatAbs(pair.diffCm) > PERP_TOL_CM) {
      stopMotors();
      if (!alignPerpendicularWithStop(stopGuardCm, F("ALIGN"))) return false;
      continue;
    }

    if (pair.avgCm <= stopGuardCm) {
      stopMotors();
      Serial.print(F("["));
      Serial.print(label);
      Serial.println(F("] Hit stop-distance guard."));
      return false;
    }

    float depthError = pair.avgCm - targetDistanceCm;
    if (floatAbs(depthError) <= targetTolCm) {
      stopMotors();
      Serial.print(F("["));
      Serial.print(label);
      Serial.println(F("] Target distance reached."));
      return true;
    }

    if (depthError > 0.0f) {
      int pwm = (pair.avgCm <= SLOW_DIST_CM) ? PWM_APPROACH_SLOW : PWM_APPROACH_FAST;
      driveForward(pwm);
      delay(APPROACH_BURST_MS);
    } else {
      driveBackward(PWM_BACKUP);
      delay(BACKUP_BURST_MS);
    }

    stopMotors();
    delay(STRAFE_SETTLE_MS);
  }

  stopMotors();
  Serial.print(F("["));
  Serial.print(label);
  Serial.println(F("] Failed to reach target distance."));
  return false;
}

bool advanceUntilAnySensorReads(float detectCm, float stopGuardCm, const __FlashStringHelper *label) {
  Serial.print(F("["));
  Serial.print(label);
  Serial.print(F("] Start forward-until-any-sensor "));
  Serial.print(detectCm, 1);
  Serial.println(F(" cm"));

  unsigned long startMs = millis();
  while ((millis() - startMs) <= INITIAL_FORWARD_TIMEOUT_MS) {
    UltraPair pair = readUltrasonicPair();
    printUltraPair(label, pair);

    bool leftHit = pair.leftValid && pair.leftCm <= detectCm;
    bool rightHit = pair.rightValid && pair.rightCm <= detectCm;

    if ((pair.leftValid && pair.leftCm <= stopGuardCm) || (pair.rightValid && pair.rightCm <= stopGuardCm)) {
      stopMotors();
      Serial.print(F("["));
      Serial.print(label);
      Serial.println(F("] Stop-distance guard hit."));
      return false;
    }

    if (leftHit || rightHit) {
      stopMotors();
      Serial.print(F("["));
      Serial.print(label);
      Serial.println(F("] Detect distance reached."));
      delay(STRAFE_SETTLE_MS);
      return true;
    }

    int pwm = (pair.valid && pair.avgCm <= SLOW_DIST_CM) ? PWM_APPROACH_SLOW : PWM_APPROACH_FAST;
    driveForward(pwm);
    delay(INITIAL_FORWARD_LOOP_DELAY_MS);
  }

  stopMotors();
  Serial.print(F("["));
  Serial.print(label);
  Serial.println(F("] Timed out before detect distance."));
  return false;
}

LdrReading readLdrNow() {
  LdrReading reading;
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

void executeTimedStrafe(bool strafeRightNow, unsigned long totalMs) {
  if (totalMs == 0UL) {
    stopMotors();
    return;
  }

  unsigned long kickMs = (totalMs < STRAFE_KICK_MS) ? totalMs : STRAFE_KICK_MS;
  unsigned long holdMs = (totalMs > kickMs) ? (totalMs - kickMs) : 0UL;

  if (strafeRightNow) {
    strafeRight(PWM_STRAFE_KICK);
    delay(kickMs);
    if (holdMs > 0UL) {
      strafeRight(PWM_STRAFE);
      delay(holdMs);
    }
  } else {
    strafeLeft(PWM_STRAFE_KICK);
    delay(kickMs);
    if (holdMs > 0UL) {
      strafeLeft(PWM_STRAFE);
      delay(holdMs);
    }
  }

  stopMotors();
}

unsigned long computeParallelStepMs(int angleErrorDeg, unsigned long minStrafeMs, unsigned long maxStrafeMs) {
  int angleAbs = abs(angleErrorDeg);

  if (angleAbs <= STEP_SHIFT_SMALL_ANGLE_DEG) return minStrafeMs;
  if (angleAbs >= STEP_SHIFT_MAX_ANGLE_DEG) return maxStrafeMs;

  unsigned long scaledMs = minStrafeMs + (unsigned long)(angleAbs - STEP_SHIFT_SMALL_ANGLE_DEG) * (maxStrafeMs - minStrafeMs) / (unsigned long)(STEP_SHIFT_MAX_ANGLE_DEG - STEP_SHIFT_SMALL_ANGLE_DEG);

  return scaledMs;
}

float computeStage1LateralShiftCm(int angleErrorDeg, float sensorWallDistCm) {
  int angleAbs = abs(angleErrorDeg);
  if (angleAbs <= 0 || sensorWallDistCm <= 0.0f) {
    return 0.0f;
  }

  float angleRad = (float)angleAbs * 3.14159265f / 180.0f;
  return sensorWallDistCm * tan(angleRad);
}

unsigned long interpolateStrafeMsFromDistance(float distanceCm, const float *distanceTableCm, const unsigned long *timeTableMs, int count) {
  if (count <= 0 || distanceCm <= 0.0f) {
    return 0UL;
  }

  if (distanceCm <= distanceTableCm[0]) {
    return (unsigned long)((distanceCm * (float)timeTableMs[0] / distanceTableCm[0]) + 0.5f);
  }

  for (int i = 1; i < count; ++i) {
    if (distanceCm <= distanceTableCm[i]) {
      float spanCm = distanceTableCm[i] - distanceTableCm[i - 1];
      if (spanCm <= 0.0f) {
        return timeTableMs[i];
      }

      float ratio = (distanceCm - distanceTableCm[i - 1]) / spanCm;
      float timeMs = (float)timeTableMs[i - 1] + ratio * (float)(timeTableMs[i] - timeTableMs[i - 1]);
      return (unsigned long)(timeMs + 0.5f);
    }
  }

  return timeTableMs[count - 1];
}

unsigned long computeRightStrafeMsForDistanceCm(float distanceCm) {
  const float distanceTableCm[] = { 1.05f, 1.45f, 1.83f, 4.60f, 20.8f, 42.8f };
  const unsigned long timeTableMs[] = { 100UL, 150UL, 200UL, 500UL, 2000UL, 4000UL };
  return interpolateStrafeMsFromDistance(distanceCm, distanceTableCm, timeTableMs, 6);
}

unsigned long computeLeftStrafeMsForDistanceCm(float distanceCm) {
  const float distanceTableCm[] = { 0.75f, 1.55f, 2.10f, 4.65f, 19.6f, 38.8f };
  const unsigned long timeTableMs[] = { 100UL, 150UL, 200UL, 500UL, 2000UL, 4000UL };
  return interpolateStrafeMsFromDistance(distanceCm, distanceTableCm, timeTableMs, 6);
}

bool runParallelStepShiftByDistance(bool strafeRightNow, int angleErrorDeg, float sensorWallDistCm, const __FlashStringHelper *label) {
  centerServo();
  delay(FULL_SCAN_SETTLE_MS);

  LdrReading reading = readLdrNow();
  printLdrReading(F("CENTER_BASE"), reading);

  float lateralShiftCm = computeStage1LateralShiftCm(angleErrorDeg, sensorWallDistCm);
  unsigned long shiftMs = strafeRightNow
                            ? computeRightStrafeMsForDistanceCm(lateralShiftCm)
                            : computeLeftStrafeMsForDistanceCm(lateralShiftCm);

  Serial.print(F("["));
  Serial.print(label);
  Serial.print(F("] dir="));
  Serial.print(strafeRightNow ? F("RIGHT") : F("LEFT"));
  Serial.print(F(" angleError="));
  Serial.print(angleErrorDeg);
  Serial.print(F(" sensorWallDistCm="));
  Serial.print(sensorWallDistCm, 1);
  Serial.print(F(" lateralShiftCm="));
  Serial.print(lateralShiftCm, 2);
  Serial.print(F(" shiftMs="));
  Serial.print(shiftMs);
  Serial.print(F(" strafePwm="));
  Serial.print(PWM_STRAFE);
  Serial.print(F(" strafeKickPwm="));
  Serial.print(PWM_STRAFE_KICK);
  Serial.print(F(" baseSum="));
  Serial.println(reading.sum);

  executeTimedStrafe(strafeRightNow, shiftMs);
  delay(STRAFE_SETTLE_MS);
  return true;
}

bool runParallelStepShift(bool strafeRightNow, int angleErrorDeg, unsigned long minStrafeMs, unsigned long maxStrafeMs, const __FlashStringHelper *label) {
  centerServo();
  delay(FULL_SCAN_SETTLE_MS);

  LdrReading reading = readLdrNow();
  printLdrReading(F("CENTER_BASE"), reading);

  unsigned long shiftMs = computeParallelStepMs(angleErrorDeg, minStrafeMs, maxStrafeMs);

  Serial.print(F("["));
  Serial.print(label);
  Serial.print(F("] dir="));
  Serial.print(strafeRightNow ? F("RIGHT") : F("LEFT"));
  Serial.print(F(" angleError="));
  Serial.print(angleErrorDeg);
  Serial.print(F(" shiftMs="));
  Serial.print(shiftMs);
  Serial.print(F(" strafePwm="));
  Serial.print(PWM_STRAFE);
  Serial.print(F(" strafeKickPwm="));
  Serial.print(PWM_STRAFE_KICK);
  Serial.print(F(" baseSum="));
  Serial.println(reading.sum);

  executeTimedStrafe(strafeRightNow, shiftMs);
  delay(STRAFE_SETTLE_MS);
  return true;
}

bool centerLightBySumPeak() {
  return centerLightBySumPeakAtDistance(TARGET_WALL_DIST_CM, TARGET_DEPTH_TOL_CM, STOP_DIST_CM, true, STAGE1_LDR_WALL_DIST_CM, STAGE1_MIN_STRAFE_MS, STAGE1_MAX_STEP_SHIFT_MS);
}

bool centerLightBySumPeakAtDistance(float targetDistanceCm, float targetTolCm, float stopGuardCm, bool useDistanceModel, float sensorWallDistCm, unsigned long minStrafeMs, unsigned long maxStrafeMs) {
  int pass = 0;
  while (true) {
    ++pass;

    if (!alignPerpendicularWithStop(stopGuardCm, F("ALIGN"))) return false;
    if (!moveToTargetDistanceWithStop(targetDistanceCm, targetTolCm, stopGuardCm, F("DEPTH"))) return false;

    ScanResult scan;
    if (pass == 1) {
      scan = runScanAndLockRange(FULL_SCAN_MIN, FULL_SCAN_MAX, FULL_SCAN_STEP_DEG, FULL_SCAN_SCORE_TIE_MARGIN, F("SCAN_FULL"));
    } else {
      scan = runScanAndLockRange(NARROW_SCAN_MIN, NARROW_SCAN_MAX, NARROW_SCAN_STEP_DEG, NARROW_SCAN_SCORE_TIE_MARGIN, F("SCAN_NARROW"));
    }
    printScanResult(F("CENTER_SCAN"), scan);
    if (!scan.found) return false;

    int angleError = scan.bestAngle - SERVO_CENTER;
    LdrReading centerReading = readLdrAtAngle(SERVO_CENTER);
    int centerGap = scan.bestScore - centerReading.sum;
    bool plateauCentered =
      (centerGap <= CENTER_SUM_ACCEPT_MARGIN) && (floatAbs(centerReading.diff) <= CENTER_DIFF_ACCEPT);

    Serial.print(F("[CENTER] pass="));
    Serial.print(pass);
    Serial.print(F(" angleError="));
    Serial.print(angleError);
    Serial.print(F(" centerSum="));
    Serial.print(centerReading.sum);
    Serial.print(F(" centerDiff="));
    Serial.print(centerReading.diff, 1);
    Serial.print(F(" centerGap="));
    Serial.print(centerGap);
    Serial.print(F(" plateauCentered="));
    Serial.println(plateauCentered ? F("true") : F("false"));

    if (abs(angleError) <= CENTER_TOL_DEG || plateauCentered) {
      Serial.println(F("[CENTER] Light is centered."));
      centerServo();
      return true;
    }

    bool strafeRightNow = POSITIVE_ANGLE_MEANS_STRAFE_RIGHT ? (angleError > 0) : (angleError < 0);
    if (useDistanceModel) {
      if (!runParallelStepShiftByDistance(strafeRightNow, angleError, sensorWallDistCm, F("PARALLEL_STEP"))) return false;
    } else {
      if (!runParallelStepShift(strafeRightNow, angleError, minStrafeMs, maxStrafeMs, F("PARALLEL_STEP"))) return false;
    }
  }

  return false;
}

bool runAutoSequence() {
  Serial.println();
  Serial.println(F("========== SECOND STAGE START =========="));

  centerServo();
  stopMotors();
  delay(300);

  if (!initialForwardUntilWallSeen()) return false;

  if (!centerLightBySumPeak()) return false;

  Serial.println(F("[FINAL] First-stage light alignment complete."));

  if (!handlePostStageColor()) return false;

  Serial.println(F("[FINAL] Second stage complete."));
  centerServo();
  stopMotors();
  return true;
}

// ============================================================
// Debug / Status
// ============================================================
void printUltraPair(const __FlashStringHelper *label, const UltraPair &pair) {
  Serial.print(F("["));
  Serial.print(label);
  Serial.print(F("] valid="));
  Serial.print(pair.valid ? F("true") : F("false"));
  Serial.print(F(" leftOk="));
  Serial.print(pair.leftValid ? F("true") : F("false"));
  Serial.print(F(" rightOk="));
  Serial.print(pair.rightValid ? F("true") : F("false"));
  Serial.print(F(" left="));
  Serial.print(pair.leftCm, 1);
  Serial.print(F(" right="));
  Serial.print(pair.rightCm, 1);
  Serial.print(F(" avg="));
  Serial.print(pair.avgCm, 1);
  Serial.print(F(" diff="));
  Serial.println(pair.diffCm, 1);
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

void printHelp() {
  Serial.println();
  Serial.println(F("=== AMR Second Stage Commands ==="));
  Serial.println(F("G : run second-stage auto sequence"));
  Serial.println(F("R : redo dark calibration"));
  Serial.println(F("S : run light scan only"));
  Serial.println(F("U : print ultrasonic pair"));
  Serial.println(F("Z : read color sensor once"));
  Serial.println(F("P : print current servo LDR reading"));
  Serial.println(F("C : center servo"));
  Serial.println(F("H or ? : help"));
  Serial.println();
}

// ============================================================
// Serial Commands
// ============================================================
void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') continue;
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');

    switch (c) {
      case 'G':
        {
          bool ok = runAutoSequence();
          Serial.print(F("[AUTO] result="));
          Serial.println(ok ? F("SUCCESS") : F("FAIL"));
          break;
        }

      case 'R':
        doDarkCalibration();
        break;

      case 'S':
        {
          ScanResult scan = runFullScanAndLock();
          printScanResult(F("MANUAL_SCAN"), scan);
          break;
        }

      case 'U':
        {
          UltraPair pair = readUltrasonicPair();
          printUltraPair(F("ULTRA"), pair);
          break;
        }

      case 'Z':
        {
          readAndClassifyColor();
          printColorReading();
          break;
        }

      case 'P':
        {
          LdrReading reading = readLdrAtAngle(servoPos);
          printLdrReading(F("NOW"), reading);
          break;
        }

      case 'C':
        centerServo();
        Serial.println(F("Servo centered."));
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

  pinMode(US_L_TRIG, OUTPUT);
  pinMode(US_L_ECHO, INPUT);
  pinMode(US_R_TRIG, OUTPUT);
  pinMode(US_R_ECHO, INPUT);
  digitalWrite(US_L_TRIG, LOW);
  digitalWrite(US_R_TRIG, LOW);

  pinMode(COLOR_S0_PIN, OUTPUT);
  pinMode(COLOR_S1_PIN, OUTPUT);
  pinMode(COLOR_S2_PIN, OUTPUT);
  pinMode(COLOR_S3_PIN, OUTPUT);
  pinMode(COLOR_OUT_PIN, INPUT);
  digitalWrite(COLOR_S0_PIN, HIGH);
  digitalWrite(COLOR_S1_PIN, LOW);

  pinMode(LDR_LEFT_PIN, INPUT);
  pinMode(LDR_RIGHT_PIN, INPUT);

  trackerServo.attach(SERVO_PIN);
  centerServo();
  stopMotors();

  Serial.println(F("ELEC3848 second-stage sketch"));
  Serial.println(F("Uses: first-stage align -> read color -> floor backup or 90deg branch turn -> side-wall align"));
  Serial.println(F("Known note: if strafe direction is reversed, flip POSITIVE_ANGLE_MEANS_STRAFE_RIGHT."));

  if (USE_PRESET_LDR_CALIBRATION) {
    loadPresetLdrCalibration();
  } else {
    doDarkCalibration();
  }
  printHelp();

  Serial.println(F("[BOOT] Auto run starts in 3 seconds..."));
  delay(3000);
}

void loop() {
  handleSerial();

  if (!autoHasRun) {
    autoHasRun = true;
    bool ok = runAutoSequence();
    Serial.print(F("[AUTO] boot result="));
    Serial.println(ok ? F("SUCCESS") : F("FAIL"));
  }
}
