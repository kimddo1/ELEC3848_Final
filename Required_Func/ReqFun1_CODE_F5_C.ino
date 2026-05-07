#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// Pin definitions
const unsigned long SERIAL_BAUD = 115200UL;
const bool ENABLE_STATUS_MONITOR = true;
const uint8_t STATUS_OLED_I2C_ADDR = 0x3C;
const uint8_t STATUS_OLED_WIDTH = 128;
const uint8_t STATUS_OLED_HEIGHT = 64;
const int8_t STATUS_OLED_RESET = -1;

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

// Tunables
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
const int FULL_SCAN_MIN = 58;
const int FULL_SCAN_MAX = 122;
const int FULL_SCAN_STEP_DEG = 5;
const int NARROW_SCAN_MIN = 74;
const int NARROW_SCAN_MAX = 106;
const int NARROW_SCAN_STEP_DEG = 2;
const int FULL_SCAN_SETTLE_MS = 120;

const int MOTOR_SIGN[4] = { -1, +1, -1, +1 };
const int MOTOR_TRIM[4] = { 0, 0, 0, 0 };
const uint8_t MOTOR_INDEX_REAR_RIGHT = 3;
const int PARK_REAR_RIGHT_BOOST_PWM = 4;
const int SIDE_BALANCE = 0;

const int PWM_INITIAL_FORWARD = 46;
const int PWM_APPROACH_FAST = 42;
const int PWM_APPROACH_SLOW = 30;
const int PWM_BACKUP = 34;
const int PWM_STRAFE = 34;
const int PWM_STRAFE_KICK = 42;
const int PWM_ALIGN_BASE = 24;
const int PWM_ALIGN_EXTRA = 12;
const int PWM_TURN_90 = 28;
const int PWM_PARK_DRIVE_FORWARD = 34;
const int PWM_PARK_DRIVE_BACKWARD = 38;
const int PWM_PARK_STRAFE = 38;
const int PWM_PARK_STRAFE_KICK = 48;
const int PWM_PARK_TURN = 34;
const int PARK_LEFT_TURN_BOOST_PWM = 6;
const int PWM_INITIAL_GUIDE_SOFT_BIAS = 5;
const int PWM_INITIAL_GUIDE_STRONG_BIAS = 8;
const unsigned long INITIAL_FORWARD_TIMEOUT_MS = 12000UL;
const unsigned long INITIAL_FORWARD_LOOP_DELAY_MS = 20UL;
const unsigned long INITIAL_FORWARD_STOP_BACKUP_MS = 1500UL;
const unsigned long INITIAL_FAIL_BACKUP_MS = 2000UL;
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
const unsigned long ALIGN_CLOSE_BACKUP_MS = 220UL;
const unsigned long TURN_LEFT_90_MS = 2400UL;
const unsigned long TURN_RIGHT_90_MS = 2550UL;

const float TARGET_WALL_DIST_CM = 35.0f;
const float INITIAL_FORWARD_DETECT_CM = 35.0f;
const float INITIAL_FORWARD_STOP_GUARD_CM = 15.0f;
const float INITIAL_FORWARD_GUIDE_DEADBAND_CM = 1.2f;
const float INITIAL_FORWARD_GUIDE_STRONG_DIFF_CM = 3.5f;
const float INITIAL_FORWARD_HIT_MARGIN_CM = 10.0f;
const float INITIAL_FORWARD_GUIDE_MAX_DIFF_CM = 18.0f;
const float COLOR_RETRY_APPROACH_CM = 25.0f;
const float TARGET_DEPTH_TOL_CM = 1.5f;
const float STAGE1_LDR_WALL_DIST_CM = 24.5f;
const float SECOND_STAGE_LDR_WALL_DIST_CM = 9.0f;
const float SECOND_STAGE_TARGET_WALL_DIST_CM = 17.0f;
const float SECOND_STAGE_TARGET_DEPTH_TOL_CM = 1.0f;
const float PARK_GREEN_FRONT_LEFT_CM = 13.4f;
const float PARK_GREEN_FRONT_RIGHT_CM = 14.6f;
const float PARK_GREEN_SIDE_CM = 22.1f;
const float PARK_RED_FRONT_LEFT_CM = 14.4f;
const float PARK_RED_FRONT_RIGHT_CM = 15.6f;
const float PARK_RED_SIDE_CM = 23.3f;
const int PARK_GREEN_DEPTH_STEER_BIAS = -3;
const int PARK_RED_DEPTH_STEER_BIAS = 0;
const float PARK_GREEN_FRONT_LEFT_TOL_CM = 0.45f;
const float PARK_GREEN_FRONT_RIGHT_TOL_CM = 0.45f;
const float PARK_GREEN_SIDE_TOL_CM = 0.40f;
const float PARK_RED_FRONT_LEFT_TOL_CM = 0.45f;
const float PARK_RED_FRONT_RIGHT_TOL_CM = 0.45f;
const float PARK_RED_SIDE_TOL_CM = 0.40f;
const float PERP_TOL_CM = 1.5f;
const float STOP_DIST_CM = 24.0f;
const float SECOND_STAGE_STOP_DIST_CM = 12.0f;
const float SLOW_DIST_CM = 38.0f;
const float PARK_COARSE_ERROR_CM = 1.5f;
const float PARK_FRONT_PARALLEL_TOL_CM = 0.65f;
const float PARK_FINAL_FRONT_PARALLEL_TOL_CM = 0.50f;
const float PARK_SIDE_PREALIGN_TOL_CM = 0.50f;
const float PARK_SIDE_FRONT_PRIORITY_TOL_CM = 0.60f;

const unsigned long PARK_ROTATE_FINE_MS = 28UL;
const unsigned long PARK_ROTATE_COARSE_MS = 50UL;
const unsigned long PARK_DRIVE_FINE_MS = 30UL;
const unsigned long PARK_DRIVE_COARSE_MS = 50UL;
const unsigned long PARK_STRAFE_FINE_MS = 35UL;
const unsigned long PARK_STRAFE_COARSE_MS = 60UL;
const unsigned long PARK_SETTLE_MS = 180UL;
const unsigned long PARK_FINAL_SETTLE_MS = 1000UL;
const int PARK_RETRIES = 80;
const int PARK_STABLE_COUNT = 3;
const int PARK_FINAL_STABLE_COUNT = 2;

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
const int ALIGN_CLOSE_RECOVERY_LIMIT = 3;
const int COLOR_NUM_SAMPLES = 10;
const int COLOR_SETTLE_MS = 5;
const unsigned long COLOR_TIMEOUT_US = 30000UL;
const unsigned long COLOR_WATCH_SAMPLE_MS = 60UL;
const int COLOR_WATCH_CONFIRM_COUNT = 3;
const unsigned long COLOR_FINAL_SAMPLE_MS = 60UL;
const int COLOR_FINAL_CONFIRM_COUNT = 3;
const int COLOR_FINAL_MAX_ATTEMPTS = 6;

const float FORWARD_RATIO[4] = { 0.9650f, 0.8462f, 0.9650f, 0.8462f };
const float BACKWARD_RATIO[4] = { 1.0000f, 0.8462f, 1.0000f, 0.8462f };
const float STRAFE_LEFT_RATIO[4] = { 1.0652f, 1.0652f, 0.9348f, 0.9348f };
const float STRAFE_RIGHT_RATIO[4] = { 1.1087f, 1.1087f, 0.8913f, 0.8913f };

// Positive angle error means the light is to the left.
const bool POSITIVE_ANGLE_MEANS_STRAFE_RIGHT = false;

// Data structures
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
  const __FlashStringHelper *name;
  bool useRightSide;
  float frontLeftCm;
  float frontRightCm;
  float sideCm;
  float frontLeftTolCm;
  float frontRightTolCm;
  float sideTolCm;
  int depthSteerBias;
};

enum ColorDecision {
  COLOR_ERROR,
  COLOR_FLOOR,
  COLOR_RED,
  COLOR_GREEN
};

// Globals
Servo trackerServo;
Adafruit_SSD1306 statusDisplay(STATUS_OLED_WIDTH, STATUS_OLED_HEIGHT, &Wire, STATUS_OLED_RESET);

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
bool stage1EarlyColorTrigger = false;
ColorDecision stage1LatchedColor = COLOR_ERROR;
int stage1ColorConfirmCount = 0;
ColorDecision stage1ColorCandidate = COLOR_ERROR;
String statusText = "";
char lastStatusText[64] = "";
bool statusMonitorReady = false;

bool autoHasRun = false;
int motorRuntimeTrim[4] = { 0, 0, 0, 0 };

// Forward declarations
void setupMotorPins();
void stopMotors();
void centerServo();
void writeServoAngle(int angle);
void setParkingMotorCompensation(bool enabled);
void initStatusMonitor();
void showStatus(String text);
void flashDetectedColor(ColorDecision color);
void showParkedScreen();
void buildStatusText(char *dest, size_t destSize, const char *line1, const char *line2);
void setStatus(const char *line1, const char *line2);
void setStatusFmt(const char *line1, const char *fmt, ...);
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
void driveForwardWithSideTrim(int basePwm, int leftTrim, int rightTrim);
void driveForwardBiased(int pwm, int steerBias);
void driveBackward(int pwm);
void driveBackwardBiased(int pwm, int steerBias);
void rotateLeft(int pwm);
void rotateRight(int pwm);
void strafeLeft(int pwm);
void strafeRight(int pwm);
float readDistanceCm(uint8_t trigPin, uint8_t echoPin);
UltraPair readUltrasonicPair();
SidePair readSideUltrasonicPair();
ParkingReadings readParkingReadings();
bool alignPerpendicular();
bool initialForwardUntilWallSeen();
void backupForRetry(const char *line1, const char *line2, const __FlashStringHelper *label);
void backupFromInitialFailure(unsigned long backupMs = INITIAL_FAIL_BACKUP_MS);
bool moveToTargetDistance();
bool alignPerpendicularWithStop(float stopGuardCm, const __FlashStringHelper *label);
bool moveToTargetDistanceWithStop(float targetDistanceCm, float targetTolCm, float stopGuardCm, const __FlashStringHelper *label);
bool advanceUntilAnySensorReads(float detectCm, float stopGuardCm, const __FlashStringHelper *label);
unsigned long readColorChannelAvg(bool s2State, bool s3State);
ColorDecision classifyColor(unsigned long rRaw, unsigned long gRaw, unsigned long bRaw);
const char *colorName(ColorDecision color);
ColorDecision readAndClassifyColor();
ColorDecision readConfirmedStationaryColor();
void printColorReading();
bool backupFromColorFloor();
void rotateNinetyDegrees(bool clockwise);
bool handlePostStageColor(bool allowRetry = true);
bool runParkingLightAssistRough();
bool runParkingFrontParallelAlign(const ParkingTarget &target);
bool runParkingSideAlign(const ParkingTarget &target);
bool runParkingFinalVerify(const ParkingTarget &target);
bool parkInAssignedZone(ColorDecision color);
bool runSecondStageLightAssist();
LdrReading readLdrNow();
void executeTimedStrafe(bool strafeRightNow, unsigned long totalMs);
void executeTimedParkingStrafe(bool strafeRightNow, unsigned long totalMs);
bool executeTimedStrafeWithColorWatch(bool strafeRightNow, unsigned long totalMs, bool watchColor);
unsigned long computeParallelStepMs(int angleErrorDeg, unsigned long minStrafeMs, unsigned long maxStrafeMs);
float computeStage1LateralShiftCm(int angleErrorDeg, float sensorWallDistCm);
unsigned long computeRightStrafeMsForDistanceCm(float distanceCm);
unsigned long computeLeftStrafeMsForDistanceCm(float distanceCm);
bool sampleColorWatchForStage1();
bool runParallelStepShiftByDistance(bool strafeRightNow, int angleErrorDeg, float sensorWallDistCm, bool watchColor, const __FlashStringHelper *label);
bool runParallelStepShift(bool strafeRightNow, int angleErrorDeg, unsigned long minStrafeMs, unsigned long maxStrafeMs, const __FlashStringHelper *label);
bool centerLightBySumPeak();
bool centerLightBySumPeakAtDistance(float targetDistanceCm, float targetTolCm, float stopGuardCm, bool useDistanceModel, float sensorWallDistCm, unsigned long minStrafeMs, unsigned long maxStrafeMs, bool watchColorOnFirstStrafe);
bool recoverStage1AfterNoColor();
bool runAutoSequence();
void printHelp();
void printUltraPair(const __FlashStringHelper *label, const UltraPair &pair);
void printParkingReadings(const __FlashStringHelper *label, const ParkingReadings &readings);
void printLdrReading(const __FlashStringHelper *label, const LdrReading &reading);
void printScanResult(const __FlashStringHelper *label, const ScanResult &scan);
float floatAbs(float value);
float clampFloat(float value, float minValue, float maxValue);
float median3(float a, float b, float c);
bool isDistanceValid(float cm);

// Helpers
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

void showStatus(String text) {
  statusText = text;

  if (!statusMonitorReady) return;

  statusDisplay.clearDisplay();
  statusDisplay.setTextSize(1);
  statusDisplay.setTextColor(SSD1306_WHITE);
  statusDisplay.setCursor(0, 0);
  statusDisplay.println(F("Status:"));
  statusDisplay.setCursor(0, 16);
  statusDisplay.println(statusText);
  statusDisplay.display();
}

void flashDetectedColor(ColorDecision color) {
  if (!statusMonitorReady) return;
  if (color != COLOR_GREEN && color != COLOR_RED) return;

  statusDisplay.clearDisplay();
  statusDisplay.fillScreen(SSD1306_WHITE);
  statusDisplay.setTextWrap(false);
  statusDisplay.setTextSize(2);
  statusDisplay.setTextColor(SSD1306_BLACK);

  if (color == COLOR_GREEN) {
    statusDisplay.setCursor(26, 16);
    statusDisplay.println(F("GREEN"));
  } else {
    statusDisplay.setCursor(40, 16);
    statusDisplay.println(F("RED"));
  }

  statusDisplay.setTextSize(1);
  statusDisplay.setCursor(22, 44);
  statusDisplay.println(F("COLOR LOCKED"));
  statusDisplay.display();
  delay(500);

  showStatus(String(lastStatusText));
}

void showParkedScreen() {
  if (!statusMonitorReady) return;

  statusDisplay.clearDisplay();
  statusDisplay.fillScreen(SSD1306_WHITE);
  statusDisplay.setTextWrap(false);
  statusDisplay.setTextColor(SSD1306_BLACK);
  statusDisplay.setTextSize(2);
  statusDisplay.setCursor(18, 22);
  statusDisplay.println(F("PARKED"));
  statusDisplay.display();
}

void buildStatusText(char *dest, size_t destSize, const char *line1, const char *line2) {
  if (destSize == 0) return;

  const char *safeLine1 = (line1 != nullptr) ? line1 : "";
  const char *safeLine2 = (line2 != nullptr) ? line2 : "";

  if (safeLine1[0] == '\0') {
    snprintf(dest, destSize, "%s", safeLine2);
  } else if (safeLine2[0] == '\0') {
    snprintf(dest, destSize, "%s", safeLine1);
  } else {
    snprintf(dest, destSize, "%s - %s", safeLine1, safeLine2);
  }
}

void setStatus(const char *line1, const char *line2) {
  char nextStatusText[64];
  buildStatusText(nextStatusText, sizeof(nextStatusText), line1, line2);

  if (strcmp(lastStatusText, nextStatusText) == 0) {
    return;
  }

  strncpy(lastStatusText, nextStatusText, sizeof(lastStatusText) - 1);
  lastStatusText[sizeof(lastStatusText) - 1] = '\0';

  Serial.print(F("[STATUS] "));
  Serial.println(lastStatusText);
  showStatus(String(lastStatusText));
}

void setStatusFmt(const char *line1, const char *fmt, ...) {
  char detail[48];
  va_list args;
  va_start(args, fmt);
  vsnprintf(detail, sizeof(detail), fmt, args);
  va_end(args);
  setStatus(line1, detail);
}

void initStatusMonitor() {
  lastStatusText[0] = '\0';
  statusText = "";
  statusMonitorReady = false;

  if (ENABLE_STATUS_MONITOR) {
    Wire.begin();

    if (statusDisplay.begin(SSD1306_SWITCHCAPVCC, STATUS_OLED_I2C_ADDR)) {
      statusMonitorReady = true;
      statusDisplay.clearDisplay();
      statusDisplay.setTextColor(SSD1306_WHITE);
      statusDisplay.setTextSize(2);
      statusDisplay.setCursor(0, 20);
      statusDisplay.println(F("OLED OK"));
      statusDisplay.display();
      delay(2000);
    } else {
      Serial.println(F("[OLED] Init failed at 0x3C; try 0x3D."));
    }
  }

  setStatus("BOOT", statusMonitorReady ? "OLED ready" : "Serial only");
}

// Motor control
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

void setParkingMotorCompensation(bool enabled) {
  // Apply the parking boost to the rear-right wheel only.
  motorRuntimeTrim[MOTOR_INDEX_REAR_RIGHT] = enabled ? PARK_REAR_RIGHT_BOOST_PWM : 0;
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
    magnitude = constrain(magnitude + MOTOR_TRIM[index] + motorRuntimeTrim[index], 0, 255);
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

void driveForwardWithSideTrim(int basePwm, int leftTrim, int rightTrim) {
  int leftPwm = constrain(basePwm - leftTrim, 0, 255);
  int rightPwm = constrain(basePwm - rightTrim, 0, 255);

  drive4(
    scaleMotionPwm(leftPwm, FORWARD_RATIO[0]),
    scaleMotionPwm(rightPwm, FORWARD_RATIO[1]),
    scaleMotionPwm(leftPwm, FORWARD_RATIO[2]),
    scaleMotionPwm(rightPwm, FORWARD_RATIO[3]));
}

void driveForwardBiased(int pwm, int steerBias) {
  int leftPwm = constrain(pwm - steerBias, 0, 255);
  int rightPwm = constrain(pwm + steerBias, 0, 255);

  drive4(
    scaleMotionPwm(leftPwm, FORWARD_RATIO[0]),
    scaleMotionPwm(rightPwm, FORWARD_RATIO[1]),
    scaleMotionPwm(leftPwm, FORWARD_RATIO[2]),
    scaleMotionPwm(rightPwm, FORWARD_RATIO[3]));
}

void driveBackward(int pwm) {
  drive4(
    -scaleMotionPwm(pwm, BACKWARD_RATIO[0]),
    -scaleMotionPwm(pwm, BACKWARD_RATIO[1]),
    -scaleMotionPwm(pwm, BACKWARD_RATIO[2]),
    -scaleMotionPwm(pwm, BACKWARD_RATIO[3]));
}

void driveBackwardBiased(int pwm, int steerBias) {
  int leftPwm = constrain(pwm - steerBias, 0, 255);
  int rightPwm = constrain(pwm + steerBias, 0, 255);

  drive4(
    -scaleMotionPwm(leftPwm, BACKWARD_RATIO[0]),
    -scaleMotionPwm(rightPwm, BACKWARD_RATIO[1]),
    -scaleMotionPwm(leftPwm, BACKWARD_RATIO[2]),
    -scaleMotionPwm(rightPwm, BACKWARD_RATIO[3]));
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

// Servo and LDR
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
  setStatus("CAL", "Preset loaded");

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
  setStatus("CAL", "Cover sensors");

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
  setStatus("CAL", "Done");
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

// Ultrasonic
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

SidePair readSideUltrasonicPair() {
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
  readings.front = readUltrasonicPair();
  delay(12);
  readings.side = readSideUltrasonicPair();
  return readings;
}

// Color sensor
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

ColorDecision readConfirmedStationaryColor() {
  setStatus("COLOR", "Reading...");
  ColorDecision candidate = COLOR_ERROR;
  int confirmCount = 0;
  ColorDecision lastSeen = COLOR_ERROR;

  for (int attempt = 1; attempt <= COLOR_FINAL_MAX_ATTEMPTS; ++attempt) {
    ColorDecision color = readAndClassifyColor();
    printColorReading();

    Serial.print(F("[COLOR_CONFIRM] attempt="));
    Serial.print(attempt);
    Serial.print(F(" decision="));
    Serial.print(colorName(color));

    if (color == COLOR_GREEN || color == COLOR_RED) {
      if (color == candidate) {
        confirmCount++;
      } else {
        candidate = color;
        confirmCount = 1;
      }
    } else {
      if (confirmCount > 0) {
        confirmCount--;
        if (confirmCount == 0) {
          candidate = COLOR_ERROR;
        }
      } else {
        candidate = COLOR_ERROR;
      }
    }

    lastSeen = color;

    Serial.print(F(" hits="));
    Serial.println(confirmCount);

    if (confirmCount >= COLOR_FINAL_CONFIRM_COUNT) {
      lastColorDecision = candidate;
      setStatus("COLOR", candidate == COLOR_GREEN ? "Found GREEN" : "Found RED");
      flashDetectedColor(candidate);
      Serial.print(F("[COLOR_CONFIRM] latched="));
      Serial.println(colorName(candidate));
      return candidate;
    }

    if (attempt < COLOR_FINAL_MAX_ATTEMPTS) {
      delay(COLOR_FINAL_SAMPLE_MS);
    }
  }

  if (lastSeen == COLOR_GREEN) {
    setStatus("COLOR", "Found GREEN");
    flashDetectedColor(COLOR_GREEN);
  } else if (lastSeen == COLOR_RED) {
    setStatus("COLOR", "Found RED");
    flashDetectedColor(COLOR_RED);
  } else {
    setStatus("COLOR", "No valid color");
  }
  return lastSeen;
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
  setStatus("COLOR", "Backup retry");
  Serial.println(F("[COLOR] FLOOR/ERROR -> backup 2 cm"));
  driveBackward(PWM_BACKUP);
  delay(FLOOR_BACKUP_MS);
  stopMotors();
  delay(STRAFE_SETTLE_MS);
  return true;
}

bool recoverStage1AfterNoColor() {
  setStatus("COLOR", "Retry scan");
  Serial.println(F("[COLOR] Retry: backup, re-approach, rescan."));

  if (!backupFromColorFloor()) return false;
  if (!advanceUntilAnySensorReads(COLOR_RETRY_APPROACH_CM, STOP_DIST_CM, F("COLOR_RETRY_APPROACH"))) return false;

  ScanResult scan = runScanAndLockRange(FULL_SCAN_MIN, FULL_SCAN_MAX, FULL_SCAN_STEP_DEG, FULL_SCAN_SCORE_TIE_MARGIN, F("SCAN_FULL"));
  printScanResult(F("RETRY_CENTER_SCAN"), scan);
  if (!scan.found) return false;

  int angleError = scan.bestAngle - SERVO_CENTER;
  LdrReading centerReading = readLdrAtAngle(SERVO_CENTER);
  int centerGap = scan.bestScore - centerReading.sum;
  bool plateauCentered =
    (centerGap <= CENTER_SUM_ACCEPT_MARGIN) && (floatAbs(centerReading.diff) <= CENTER_DIFF_ACCEPT);

  Serial.print(F("[RETRY_CENTER] angleError="));
  Serial.print(angleError);
  Serial.print(F(" centerSum="));
  Serial.print(centerReading.sum);
  Serial.print(F(" centerDiff="));
  Serial.print(centerReading.diff, 1);
  Serial.print(F(" centerGap="));
  Serial.print(centerGap);
  Serial.print(F(" plateauCentered="));
  Serial.println(plateauCentered ? F("true") : F("false"));

  if (!(abs(angleError) <= CENTER_TOL_DEG || plateauCentered)) {
    bool strafeRightNow = POSITIVE_ANGLE_MEANS_STRAFE_RIGHT ? (angleError > 0) : (angleError < 0);
    if (!runParallelStepShiftByDistance(strafeRightNow, angleError, STAGE1_LDR_WALL_DIST_CM, true, F("RETRY_PARALLEL_STEP"))) {
      if (stage1EarlyColorTrigger) {
        Serial.println(F("[COLOR] Early trigger during retry strafe."));
        return true;
      }
      return false;
    }
  }

  if (stage1EarlyColorTrigger) {
    Serial.println(F("[COLOR] Early trigger during retry."));
    return true;
  }

  return centerLightBySumPeakAtDistance(TARGET_WALL_DIST_CM, TARGET_DEPTH_TOL_CM, STOP_DIST_CM, true, STAGE1_LDR_WALL_DIST_CM, STAGE1_MIN_STRAFE_MS, STAGE1_MAX_STEP_SHIFT_MS, true);
}

void rotateNinetyDegrees(bool clockwise) {
  setStatus("TURN", clockwise ? "CW 90 deg" : "CCW 90 deg");
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

bool runSecondStageLightAssist() {
  setStatus("SECOND", "Light assist");
  Serial.println(F("[SECOND_LIGHT] Start one-pass light assist."));
  centerServo();
  stopMotors();
  delay(FULL_SCAN_SETTLE_MS);

  ScanResult scan = runScanAndLockRange(FULL_SCAN_MIN, FULL_SCAN_MAX, FULL_SCAN_STEP_DEG, FULL_SCAN_SCORE_TIE_MARGIN, F("SIDE_SCAN_FULL"));
  printScanResult(F("SIDE_CENTER_SCAN"), scan);

  if (!scan.found) {
    setStatus("SECOND", "Skip light");
    Serial.println(F("[SECOND_LIGHT] No peak; skip assist."));
    centerServo();
    return true;
  }

  int angleError = scan.bestAngle - SERVO_CENTER;
  LdrReading centerReading = readLdrAtAngle(SERVO_CENTER);
  int centerGap = scan.bestScore - centerReading.sum;
  bool plateauCentered =
    (centerGap <= CENTER_SUM_ACCEPT_MARGIN) && (floatAbs(centerReading.diff) <= CENTER_DIFF_ACCEPT);

  Serial.print(F("[SECOND_LIGHT] angleError="));
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
    setStatus("SECOND", "Light centered");
    Serial.println(F("[SECOND_LIGHT] Already centered."));
    centerServo();
    return true;
  }

  bool strafeRightNow = POSITIVE_ANGLE_MEANS_STRAFE_RIGHT ? (angleError > 0) : (angleError < 0);
  if (!runParallelStepShiftByDistance(strafeRightNow, angleError, SECOND_STAGE_LDR_WALL_DIST_CM, false, F("SIDE_PARALLEL_STEP"))) {
    setStatus("FAIL", "Second assist");
    Serial.println(F("[SECOND_LIGHT] Parallel assist failed."));
    return false;
  }

  setStatus("SECOND", "Hand to park");
  Serial.println(F("[SECOND_LIGHT] Assist done; hand off to parking."));
  return true;
}

bool runParkingLightAssistRough() {
  setStatus("PARK", "LDR center");
  Serial.println(F("[PARK_LDR] Start rough LDR centering."));
  centerServo();
  stopMotors();
  delay(FULL_SCAN_SETTLE_MS);

  ScanResult scan = runScanAndLockRange(NARROW_SCAN_MIN, NARROW_SCAN_MAX, NARROW_SCAN_STEP_DEG, NARROW_SCAN_SCORE_TIE_MARGIN, F("PARK_SCAN"));
  printScanResult(F("PARK_LDR_SCAN"), scan);

  if (!scan.found) {
    setStatus("PARK", "Skip LDR");
    Serial.println(F("[PARK_LDR] No peak; skip centering."));
    centerServo();
    return true;
  }

  int angleError = scan.bestAngle - SERVO_CENTER;
  LdrReading centerReading = readLdrAtAngle(SERVO_CENTER);
  int centerGap = scan.bestScore - centerReading.sum;
  bool plateauCentered =
    (centerGap <= CENTER_SUM_ACCEPT_MARGIN) && (floatAbs(centerReading.diff) <= CENTER_DIFF_ACCEPT);

  Serial.print(F("[PARK_LDR] angleError="));
  Serial.print(angleError);
  Serial.print(F(" centerSum="));
  Serial.print(centerReading.sum);
  Serial.print(F(" centerDiff="));
  Serial.print(centerReading.diff, 1);
  Serial.print(F(" centerGap="));
  Serial.print(centerGap);
  Serial.print(F(" plateauCentered="));
  Serial.println(plateauCentered ? F("true") : F("false"));

  if (abs(angleError) <= FINAL_CENTER_TOL_DEG || plateauCentered) {
    setStatus("PARK", "LDR ready");
    Serial.println(F("[PARK_LDR] Centering already acceptable."));
    centerServo();
    return true;
  }

  bool strafeRightNow = POSITIVE_ANGLE_MEANS_STRAFE_RIGHT ? (angleError > 0) : (angleError < 0);
  if (!runParallelStepShift(strafeRightNow, angleError, STAGE2_MIN_STRAFE_MS, STAGE2_MAX_STEP_SHIFT_MS, F("PARK_LDR_STEP"))) {
    setStatus("FAIL", "Park LDR");
    Serial.println(F("[PARK_LDR] Rough LDR centering failed."));
    return false;
  }

  setStatus("PARK", "LDR ready");
  Serial.println(F("[PARK_LDR] Rough LDR centering complete."));
  return true;
}

bool runParkingFrontParallelAlign(const ParkingTarget &target) {
  setStatus("PARK", "Front parallel");
  Serial.println(F("[PARK_FRONT] Start front parallel alignment."));

  const float targetDiff = target.frontLeftCm - target.frontRightCm;
  int stableCount = 0;

  while (true) {
    UltraPair pair = readUltrasonicPair();
    printUltraPair(F("PARK_FRONT"), pair);

    if (!pair.valid) {
      stableCount = 0;
      stopMotors();
      Serial.println(F("[PARK_FRONT] Front ultrasonic invalid."));
      delay(PARK_SETTLE_MS);
      continue;
    }

    float diffError = pair.diffCm - targetDiff;
    Serial.print(F("[PARK_FRONT] diffError="));
    Serial.println(diffError, 2);

    if (floatAbs(diffError) <= PARK_FRONT_PARALLEL_TOL_CM) {
      stableCount++;
      if (stableCount >= PARK_STABLE_COUNT) {
        stopMotors();
        setStatus("PARK", "Front ready");
        Serial.println(F("[PARK_FRONT] Front aligned."));
        return true;
      }
      delay(PARK_SETTLE_MS);
      continue;
    }

    stableCount = 0;

    unsigned long burstMs = (floatAbs(diffError) > PARK_COARSE_ERROR_CM) ? PARK_ROTATE_COARSE_MS : PARK_ROTATE_FINE_MS;
    if (diffError > 0.0f) {
      Serial.print(F("[PARK_FRONT] rotateLeft burstMs="));
      Serial.println(burstMs);
      rotateLeft(PWM_PARK_TURN + PARK_LEFT_TURN_BOOST_PWM);
    } else {
      Serial.print(F("[PARK_FRONT] rotateRight burstMs="));
      Serial.println(burstMs);
      rotateRight(PWM_PARK_TURN);
    }

    delay(burstMs);
    stopMotors();
    delay(PARK_SETTLE_MS);
  }

  return false;
}

bool runParkingSideAlign(const ParkingTarget &target) {
  setStatus("PARK", "Side align");
  Serial.println(F("[PARK_SIDE] Start side-distance alignment."));

  int stableCount = 0;

  while (true) {
    SidePair pair = readSideUltrasonicPair();
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
      Serial.println(F("[PARK_SIDE] Side ultrasonic invalid."));
      delay(PARK_SETTLE_MS);
      continue;
    }

    float sideError = activeSideCm - target.sideCm;
    Serial.print(F("[PARK_SIDE] sideError="));
    Serial.println(sideError, 2);

    if (floatAbs(sideError) <= PARK_SIDE_PREALIGN_TOL_CM) {
      stableCount++;
      if (stableCount >= PARK_STABLE_COUNT) {
        stopMotors();
        setStatus("PARK", "Side ready");
        Serial.println(F("[PARK_SIDE] Side alignment complete."));
        return true;
      }
      delay(PARK_SETTLE_MS);
      continue;
    }

    stableCount = 0;

    unsigned long burstMs = (floatAbs(sideError) > PARK_COARSE_ERROR_CM) ? PARK_STRAFE_COARSE_MS : PARK_STRAFE_FINE_MS;
    bool strafeTowardTarget = sideError > 0.0f ? target.useRightSide : !target.useRightSide;

    Serial.print(F("[PARK_SIDE] strafe "));
    Serial.print(strafeTowardTarget ? F("RIGHT") : F("LEFT"));
    Serial.print(F(" burstMs="));
    Serial.println(burstMs);
    executeTimedParkingStrafe(strafeTowardTarget, burstMs);
    stopMotors();
    delay(PARK_SETTLE_MS);
  }

  return false;
}

bool runParkingFinalVerify(const ParkingTarget &target) {
  setStatus("PARK", "Final verify");
  Serial.println(F("[PARK_FINAL] Start verification."));

  const float targetDiff = target.frontLeftCm - target.frontRightCm;
  int stableCount = 0;

  while (true) {
    stopMotors();
    delay(PARK_FINAL_SETTLE_MS);

    ParkingReadings readings = readParkingReadings();
    printParkingReadings(F("PARK_FINAL"), readings);

    if (!readings.front.leftValid || !readings.front.rightValid) {
      stableCount = 0;
      stopMotors();
      Serial.println(F("[PARK_FINAL] Front ultrasonic invalid."));
      continue;
    }

    bool activeSideValid = target.useRightSide ? readings.side.rightValid : readings.side.leftValid;
    float activeSideCm = target.useRightSide ? readings.side.rightCm : readings.side.leftCm;
    if (!activeSideValid) {
      stableCount = 0;
      stopMotors();
      Serial.println(F("[PARK_FINAL] Side ultrasonic invalid."));
      continue;
    }

    float errorFL = readings.front.leftCm - target.frontLeftCm;
    float errorFR = readings.front.rightCm - target.frontRightCm;
    float sideError = activeSideCm - target.sideCm;
    float diffError = readings.front.diffCm - targetDiff;

    bool leftHigh = errorFL > target.frontLeftTolCm;
    bool leftLow = errorFL < -target.frontLeftTolCm;
    bool rightHigh = errorFR > target.frontRightTolCm;
    bool rightLow = errorFR < -target.frontRightTolCm;
    bool sideHigh = sideError > target.sideTolCm;
    bool sideLow = sideError < -target.sideTolCm;
    bool frontParallelOk = floatAbs(diffError) <= PARK_FINAL_FRONT_PARALLEL_TOL_CM;
    bool frontDepthOk = !leftHigh && !leftLow && !rightHigh && !rightLow;
    bool sideOk = !sideHigh && !sideLow;
    bool sideRoughOk = floatAbs(sideError) <= PARK_SIDE_FRONT_PRIORITY_TOL_CM;

    Serial.print(F("[PARK_FINAL_ERR] eFL="));
    Serial.print(errorFL, 2);
    Serial.print(F(" eFR="));
    Serial.print(errorFR, 2);
    Serial.print(F(" eSide="));
    Serial.print(sideError, 2);
    Serial.print(F(" eDiff="));
    Serial.println(diffError, 2);

    if (frontParallelOk && frontDepthOk && sideOk) {
      stableCount++;
      Serial.print(F("[PARK_FINAL] Within tolerance. stableCount="));
      Serial.println(stableCount);
      if (stableCount >= PARK_FINAL_STABLE_COUNT) {
        stopMotors();
        setStatus("PARK", "Aligned");
        Serial.println(F("[PARK_FINAL] Final parking verified."));
        return true;
      }
      continue;
    }

    stableCount = 0;

    if (!sideOk && !sideRoughOk) {
      unsigned long burstMs = (floatAbs(sideError) > PARK_COARSE_ERROR_CM) ? PARK_STRAFE_COARSE_MS : PARK_STRAFE_FINE_MS;
      bool strafeTowardTarget = sideError > 0.0f ? target.useRightSide : !target.useRightSide;
      setStatus("PARK", "Fix side");
      Serial.print(F("[PARK_FINAL] strafe "));
      Serial.print(strafeTowardTarget ? F("RIGHT") : F("LEFT"));
      Serial.print(F(" burstMs="));
      Serial.println(burstMs);
      executeTimedParkingStrafe(strafeTowardTarget, burstMs);
    } else if (!frontParallelOk) {
      unsigned long burstMs = (floatAbs(diffError) > PARK_COARSE_ERROR_CM) ? PARK_ROTATE_COARSE_MS : PARK_ROTATE_FINE_MS;
      setStatus("PARK", "Fix front");
      if (diffError > 0.0f) {
        Serial.print(F("[PARK_FINAL] rotateLeft burstMs="));
        Serial.println(burstMs);
        rotateLeft(PWM_PARK_TURN + PARK_LEFT_TURN_BOOST_PWM);
      } else {
        Serial.print(F("[PARK_FINAL] rotateRight burstMs="));
        Serial.println(burstMs);
        rotateRight(PWM_PARK_TURN);
      }
      delay(burstMs);
    } else if (leftHigh && rightHigh) {
      unsigned long burstMs = ((floatAbs(errorFL) > PARK_COARSE_ERROR_CM) || (floatAbs(errorFR) > PARK_COARSE_ERROR_CM))
                                ? PARK_DRIVE_COARSE_MS : PARK_DRIVE_FINE_MS;
      int steerBias = target.depthSteerBias;
      setStatus("PARK", "Fix depth");
      Serial.print(F("[PARK_FINAL] forward burstMs="));
      Serial.print(burstMs);
      Serial.print(F(" steerBias="));
      Serial.println(steerBias);
      if (steerBias == 0) {
        driveForward(PWM_PARK_DRIVE_FORWARD);
      } else {
        driveForwardBiased(PWM_PARK_DRIVE_FORWARD, steerBias);
      }
      delay(burstMs);
    } else if (leftLow && rightLow) {
      unsigned long burstMs = ((floatAbs(errorFL) > PARK_COARSE_ERROR_CM) || (floatAbs(errorFR) > PARK_COARSE_ERROR_CM))
                                ? PARK_DRIVE_COARSE_MS : PARK_DRIVE_FINE_MS;
      int steerBias = -target.depthSteerBias;
      setStatus("PARK", "Fix depth");
      Serial.print(F("[PARK_FINAL] backward burstMs="));
      Serial.print(burstMs);
      Serial.print(F(" steerBias="));
      Serial.println(steerBias);
      if (steerBias == 0) {
        driveBackward(PWM_PARK_DRIVE_BACKWARD);
      } else {
        driveBackwardBiased(PWM_PARK_DRIVE_BACKWARD, steerBias);
      }
      delay(burstMs);
    } else if (leftHigh && !rightHigh) {
      unsigned long burstMs = (floatAbs(errorFL) > PARK_COARSE_ERROR_CM) ? PARK_ROTATE_COARSE_MS : PARK_ROTATE_FINE_MS;
      setStatus("PARK", "Fix front");
      Serial.print(F("[PARK_FINAL] rotateLeft burstMs="));
      Serial.println(burstMs);
      rotateLeft(PWM_PARK_TURN + PARK_LEFT_TURN_BOOST_PWM);
      delay(burstMs);
    } else if (rightHigh && !leftHigh) {
      unsigned long burstMs = (floatAbs(errorFR) > PARK_COARSE_ERROR_CM) ? PARK_ROTATE_COARSE_MS : PARK_ROTATE_FINE_MS;
      setStatus("PARK", "Fix front");
      Serial.print(F("[PARK_FINAL] rotateRight burstMs="));
      Serial.println(burstMs);
      rotateRight(PWM_PARK_TURN);
      delay(burstMs);
    } else if (leftLow && !rightLow) {
      unsigned long burstMs = (floatAbs(errorFL) > PARK_COARSE_ERROR_CM) ? PARK_ROTATE_COARSE_MS : PARK_ROTATE_FINE_MS;
      setStatus("PARK", "Fix front");
      Serial.print(F("[PARK_FINAL] rotateRight burstMs="));
      Serial.println(burstMs);
      rotateRight(PWM_PARK_TURN);
      delay(burstMs);
    } else if (rightLow && !leftLow) {
      unsigned long burstMs = (floatAbs(errorFR) > PARK_COARSE_ERROR_CM) ? PARK_ROTATE_COARSE_MS : PARK_ROTATE_FINE_MS;
      setStatus("PARK", "Fix front");
      Serial.print(F("[PARK_FINAL] rotateLeft burstMs="));
      Serial.println(burstMs);
      rotateLeft(PWM_PARK_TURN + PARK_LEFT_TURN_BOOST_PWM);
      delay(burstMs);
    } else if (!sideOk) {
      unsigned long burstMs = (floatAbs(sideError) > PARK_COARSE_ERROR_CM) ? PARK_STRAFE_COARSE_MS : PARK_STRAFE_FINE_MS;
      bool strafeTowardTarget = sideError > 0.0f ? target.useRightSide : !target.useRightSide;
      setStatus("PARK", "Fix side");
      Serial.print(F("[PARK_FINAL] strafe "));
      Serial.print(strafeTowardTarget ? F("RIGHT") : F("LEFT"));
      Serial.print(F(" burstMs="));
      Serial.println(burstMs);
      executeTimedParkingStrafe(strafeTowardTarget, burstMs);
    }

    stopMotors();
  }

  return false;
}

bool parkInAssignedZone(ColorDecision color) {
  ParkingTarget target;

  if (color == COLOR_GREEN) {
    target.name = F("GREEN");
    target.useRightSide = true;
    target.frontLeftCm = PARK_GREEN_FRONT_LEFT_CM;
    target.frontRightCm = PARK_GREEN_FRONT_RIGHT_CM;
    target.sideCm = PARK_GREEN_SIDE_CM;
    target.frontLeftTolCm = PARK_GREEN_FRONT_LEFT_TOL_CM;
    target.frontRightTolCm = PARK_GREEN_FRONT_RIGHT_TOL_CM;
    target.sideTolCm = PARK_GREEN_SIDE_TOL_CM;
    target.depthSteerBias = PARK_GREEN_DEPTH_STEER_BIAS;
  } else if (color == COLOR_RED) {
    target.name = F("RED");
    target.useRightSide = false;
    target.frontLeftCm = PARK_RED_FRONT_LEFT_CM;
    target.frontRightCm = PARK_RED_FRONT_RIGHT_CM;
    target.sideCm = PARK_RED_SIDE_CM;
    target.frontLeftTolCm = PARK_RED_FRONT_LEFT_TOL_CM;
    target.frontRightTolCm = PARK_RED_FRONT_RIGHT_TOL_CM;
    target.sideTolCm = PARK_RED_SIDE_TOL_CM;
    target.depthSteerBias = PARK_RED_DEPTH_STEER_BIAS;
  } else {
    setStatus("FAIL", "Bad color");
    Serial.println(F("[PARK] Invalid parking color."));
    return false;
  }

  setStatus("PARK", color == COLOR_GREEN ? "Target GREEN" : "Target RED");
  setParkingMotorCompensation(true);

  Serial.print(F("[PARK] Start ultrasonic parking for "));
  Serial.println(target.name);
  Serial.print(F("[PARK] rearRightBoostPwm="));
  Serial.println(PARK_REAR_RIGHT_BOOST_PWM);
  Serial.print(F("[PARK] driveFwd="));
  Serial.print(PWM_PARK_DRIVE_FORWARD);
  Serial.print(F(" driveBack="));
  Serial.print(PWM_PARK_DRIVE_BACKWARD);
  Serial.print(F(" strafe="));
  Serial.print(PWM_PARK_STRAFE);
  Serial.print(F(" turn="));
  Serial.println(PWM_PARK_TURN);
  Serial.print(F("[PARK] targetFL="));
  Serial.print(target.frontLeftCm, 1);
  Serial.print(F(" targetFR="));
  Serial.print(target.frontRightCm, 1);
  Serial.print(F(" targetSide="));
  Serial.print(target.sideCm, 1);
  Serial.print(F(" sideSensor="));
  Serial.println(target.useRightSide ? F("RIGHT") : F("LEFT"));

  if (!runParkingLightAssistRough()) {
    setParkingMotorCompensation(false);
    return false;
  }

  if (!runParkingSideAlign(target)) {
    setParkingMotorCompensation(false);
    return false;
  }

  if (!runParkingFrontParallelAlign(target)) {
    setParkingMotorCompensation(false);
    return false;
  }

  bool aligned = runParkingFinalVerify(target);
  setParkingMotorCompensation(false);
  if (!aligned) return false;

  showParkedScreen();
  Serial.println(F("[PARK] Parking aligned."));
  return true;
}

bool handlePostStageColor(bool allowRetry) {
  centerServo();
  stopMotors();
  delay(FULL_SCAN_SETTLE_MS);

  ColorDecision color = COLOR_ERROR;
  if (stage1EarlyColorTrigger && (stage1LatchedColor == COLOR_GREEN || stage1LatchedColor == COLOR_RED)) {
    color = stage1LatchedColor;
    lastColorDecision = color;
    setStatus("COLOR", color == COLOR_GREEN ? "Found GREEN" : "Found RED");
    flashDetectedColor(color);
    Serial.print(F("[COLOR] Using latched stage-1 detection: "));
    Serial.println(colorName(color));
  } else {
    color = readConfirmedStationaryColor();
  }

  stage1EarlyColorTrigger = false;
  stage1LatchedColor = COLOR_ERROR;
  stage1ColorConfirmCount = 0;
  stage1ColorCandidate = COLOR_ERROR;

  if (color == COLOR_FLOOR || color == COLOR_ERROR) {
    if (!allowRetry) {
      setStatus("FAIL", "Color not found");
      Serial.println(F("[COLOR] No valid color after retry."));
      return false;
    }

    if (!recoverStage1AfterNoColor()) return false;
    return handlePostStageColor(false);
  }

  setStatus("TURN", color == COLOR_GREEN ? "GREEN route" : "RED route");
  rotateNinetyDegrees(color == COLOR_GREEN);

  if (!alignPerpendicularWithStop(SECOND_STAGE_STOP_DIST_CM, F("SIDE_ALIGN"))) return false;
  if (!advanceUntilAnySensorReads(SECOND_STAGE_TARGET_WALL_DIST_CM, SECOND_STAGE_STOP_DIST_CM, F("SIDE_APPROACH"))) return false;
  if (!parkInAssignedZone(color)) return false;

  setStatus("DONE", "Parking complete");
  showParkedScreen();
  Serial.println(F("[SECOND] Parking zone reached and aligned."));
  return true;
}

// Control phases
void backupForRetry(const char *line1, const char *line2, const __FlashStringHelper *label) {
  setStatus(line1, line2);
  Serial.print(F("["));
  Serial.print(label);
  Serial.print(F("] Recovery backup for "));
  Serial.print(INITIAL_FAIL_BACKUP_MS);
  Serial.println(F(" ms"));
  driveBackward(PWM_BACKUP);
  delay(INITIAL_FAIL_BACKUP_MS);
  stopMotors();
  delay(STRAFE_SETTLE_MS);
}

void backupFromInitialFailure(unsigned long backupMs) {
  setStatus("INITIAL", "Backup");
  Serial.print(F("[INITIAL] Recovery backup for "));
  Serial.print(backupMs);
  Serial.println(F(" ms"));
  driveBackward(PWM_BACKUP);
  delay(backupMs);
  stopMotors();
  delay(STRAFE_SETTLE_MS);
}

bool initialForwardUntilWallSeen() {
  while (true) {
    setStatus("INITIAL", "Find wall");
    Serial.println(F("[INITIAL] Drive until first 35 cm hit."));

    unsigned long startMs = millis();
    bool guidePhaseAnnounced = false;
    bool retryStage = false;
    driveForward(PWM_INITIAL_FORWARD);

    while ((millis() - startMs) <= INITIAL_FORWARD_TIMEOUT_MS) {
      UltraPair pair = readUltrasonicPair();
      printUltraPair(F("INITIAL"), pair);

      bool frontWallSeen = false;
      if (pair.leftValid && pair.rightValid) {
        bool bothWithinDetect = pair.avgCm <= INITIAL_FORWARD_DETECT_CM;
        bool pairedHit =
          (pair.leftCm <= INITIAL_FORWARD_DETECT_CM && pair.rightCm <= (INITIAL_FORWARD_DETECT_CM + INITIAL_FORWARD_HIT_MARGIN_CM)) ||
          (pair.rightCm <= INITIAL_FORWARD_DETECT_CM && pair.leftCm <= (INITIAL_FORWARD_DETECT_CM + INITIAL_FORWARD_HIT_MARGIN_CM));
        frontWallSeen = bothWithinDetect || pairedHit;
      }

      if ((pair.leftValid && pair.leftCm <= INITIAL_FORWARD_STOP_GUARD_CM) ||
          (pair.rightValid && pair.rightCm <= INITIAL_FORWARD_STOP_GUARD_CM)) {
        stopMotors();
        Serial.println(F("[INITIAL] Stop guard hit; backup."));
        backupFromInitialFailure(INITIAL_FORWARD_STOP_BACKUP_MS);
        retryStage = true;
        break;
      }

      if (frontWallSeen) {
        stopMotors();
        setStatus("INITIAL", "Wall found");
        Serial.println(F("[INITIAL] First wall detection reached."));
        delay(STRAFE_SETTLE_MS);
        return true;
      }

      int leftTrim = 0;
      int rightTrim = 0;

      if (!guidePhaseAnnounced) {
        guidePhaseAnnounced = true;
        setStatus("INITIAL", "Guide wall");
        Serial.println(F("[INITIAL] Front trim active."));
      }

      if (pair.valid && floatAbs(pair.diffCm) <= INITIAL_FORWARD_GUIDE_MAX_DIFF_CM) {
        float diffAbs = floatAbs(pair.diffCm);
        if (diffAbs > INITIAL_FORWARD_GUIDE_DEADBAND_CM) {
          int trim = (diffAbs >= INITIAL_FORWARD_GUIDE_STRONG_DIFF_CM)
            ? PWM_INITIAL_GUIDE_STRONG_BIAS
            : PWM_INITIAL_GUIDE_SOFT_BIAS;

          if (pair.diffCm < 0.0f) {
            leftTrim = trim;
          } else {
            rightTrim = trim;
          }
        }
      }

      if (leftTrim == 0 && rightTrim == 0) {
        driveForward(PWM_INITIAL_FORWARD);
      } else {
        driveForwardWithSideTrim(PWM_INITIAL_FORWARD, leftTrim, rightTrim);
      }
      delay(INITIAL_FORWARD_LOOP_DELAY_MS);
    }

    if (retryStage) {
      continue;
    }

    stopMotors();
    Serial.println(F("[INITIAL] Timeout before 35 cm hit; backup."));
    backupFromInitialFailure();
  }
}

bool alignPerpendicular() {
  return alignPerpendicularWithStop(STOP_DIST_CM, F("ALIGN"));
}

bool alignPerpendicularWithStop(float stopGuardCm, const __FlashStringHelper *label) {
  while (true) {
    setStatus("ALIGN", "Perpendicular");
    Serial.print(F("["));
    Serial.print(label);
    Serial.println(F("] Start perpendicular alignment"));
    int stableCount = 0;
    int lastTurnDir = 0;
    int lastPwm = -1;
    bool retryStage = false;

    for (int attempt = 0; attempt < ALIGN_RETRIES; ++attempt) {
      UltraPair pair = readUltrasonicPair();
      printUltraPair(label, pair);

      if (!pair.valid) {
        stableCount = 0;

        if ((pair.leftValid && pair.leftCm <= stopGuardCm) || (pair.rightValid && pair.rightCm <= stopGuardCm)) {
          stopMotors();
          Serial.print(F("["));
          Serial.print(label);
          Serial.println(F("] One sensor too close; backup."));
          backupForRetry("ALIGN", "Backup retry", label);
          retryStage = true;
          break;
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
        Serial.println(F("] Too close to wall. Backup and retry."));
        backupForRetry("ALIGN", "Backup retry", label);
        retryStage = true;
        break;
      }

      if (floatAbs(pair.diffCm) <= PERP_TOL_CM) {
        stableCount++;
        if (stableCount >= ALIGN_STABLE_COUNT) {
          stopMotors();
          setStatus("ALIGN", "Aligned");
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

    if (retryStage) {
      continue;
    }

    stopMotors();
    Serial.print(F("["));
    Serial.print(label);
    Serial.println(F("] Failed to converge. Backup and retry."));
    backupForRetry("ALIGN", "Backup retry", label);
  }
}

bool moveToTargetDistance() {
  return moveToTargetDistanceWithStop(TARGET_WALL_DIST_CM, TARGET_DEPTH_TOL_CM, STOP_DIST_CM, F("DEPTH"));
}

bool moveToTargetDistanceWithStop(float targetDistanceCm, float targetTolCm, float stopGuardCm, const __FlashStringHelper *label) {
  while (true) {
    setStatus("DEPTH", "Target distance");
    Serial.print(F("["));
    Serial.print(label);
    Serial.print(F("] Start approach to "));
    Serial.print(targetDistanceCm, 1);
    Serial.println(F(" cm"));

    bool retryStage = false;

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
        Serial.println(F("] Hit stop-distance guard. Backup and retry."));
        backupForRetry("DEPTH", "Backup retry", label);
        retryStage = true;
        break;
      }

      float depthError = pair.avgCm - targetDistanceCm;
      if (floatAbs(depthError) <= targetTolCm) {
        stopMotors();
        setStatus("DEPTH", "Distance OK");
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

    if (retryStage) {
      continue;
    }

    stopMotors();
    Serial.print(F("["));
    Serial.print(label);
    Serial.println(F("] Failed to reach target distance; backup."));
    backupForRetry("DEPTH", "Backup retry", label);
  }
}

bool advanceUntilAnySensorReads(float detectCm, float stopGuardCm, const __FlashStringHelper *label) {
  while (true) {
    setStatus("APPROACH", "Move forward");
    Serial.print(F("["));
    Serial.print(label);
    Serial.print(F("] Start forward-until-any-sensor "));
    Serial.print(detectCm, 1);
    Serial.println(F(" cm"));

    unsigned long startMs = millis();
    bool retryStage = false;
    while ((millis() - startMs) <= INITIAL_FORWARD_TIMEOUT_MS) {
      UltraPair pair = readUltrasonicPair();
      printUltraPair(label, pair);

      bool leftHit = pair.leftValid && pair.leftCm <= detectCm;
      bool rightHit = pair.rightValid && pair.rightCm <= detectCm;

      if ((pair.leftValid && pair.leftCm <= stopGuardCm) || (pair.rightValid && pair.rightCm <= stopGuardCm)) {
        stopMotors();
        Serial.print(F("["));
        Serial.print(label);
        Serial.println(F("] Stop-distance guard hit. Backup and retry."));
        backupForRetry("APPROACH", "Backup retry", label);
        retryStage = true;
        break;
      }

      if (leftHit || rightHit) {
        stopMotors();
        setStatus("APPROACH", "Distance hit");
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

    if (retryStage) {
      continue;
    }

    stopMotors();
    Serial.print(F("["));
    Serial.print(label);
    Serial.println(F("] Timeout before detect distance; backup."));
    backupForRetry("APPROACH", "Backup retry", label);
  }
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

void executeTimedParkingStrafe(bool strafeRightNow, unsigned long totalMs) {
  if (totalMs == 0UL) {
    stopMotors();
    return;
  }

  unsigned long kickMs = (totalMs < STRAFE_KICK_MS) ? totalMs : STRAFE_KICK_MS;
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
}

bool sampleColorWatchForStage1() {
  ColorDecision color = readAndClassifyColor();
  Serial.print(F("[COLOR_WATCH] decision="));
  Serial.print(colorName(color));
  Serial.print(F(" rg="));
  Serial.print(lastColorRgRatio, 3);

  if (color == COLOR_GREEN || color == COLOR_RED) {
    if (color == stage1ColorCandidate) {
      stage1ColorConfirmCount++;
    } else {
      stage1ColorCandidate = color;
      stage1ColorConfirmCount = 1;
    }
  } else {
    if (stage1ColorConfirmCount > 0) {
      stage1ColorConfirmCount--;
      if (stage1ColorConfirmCount == 0) {
        stage1ColorCandidate = COLOR_ERROR;
      }
    } else {
      stage1ColorCandidate = COLOR_ERROR;
    }
  }

  Serial.print(F(" hits="));
  Serial.println(stage1ColorConfirmCount);

  if (stage1ColorConfirmCount >= COLOR_WATCH_CONFIRM_COUNT) {
    stage1EarlyColorTrigger = true;
    stage1LatchedColor = stage1ColorCandidate;
    stage1ColorConfirmCount = 0;
    stage1ColorCandidate = COLOR_ERROR;
    setStatus("COLOR", stage1LatchedColor == COLOR_GREEN ? "Found GREEN" : "Found RED");

    Serial.print(F("[COLOR_WATCH] latched="));
    Serial.println(colorName(stage1LatchedColor));
    return true;
  }

  return false;
}

bool executeTimedStrafeWithColorWatch(bool strafeRightNow, unsigned long totalMs, bool watchColor) {
  if (totalMs == 0UL) {
    stopMotors();
    return false;
  }

  unsigned long elapsedMs = 0UL;
  unsigned long sampleAccumMs = 0UL;

  while (elapsedMs < totalMs) {
    bool inKickPhase = (elapsedMs < STRAFE_KICK_MS);
    unsigned long phaseRemainingMs = inKickPhase ? (STRAFE_KICK_MS - elapsedMs) : (totalMs - elapsedMs);
    unsigned long stepMs = phaseRemainingMs;

    if (watchColor) {
      unsigned long untilSampleMs = COLOR_WATCH_SAMPLE_MS - sampleAccumMs;
      if (stepMs > untilSampleMs) stepMs = untilSampleMs;
    }

    unsigned long totalRemainingMs = totalMs - elapsedMs;
    if (stepMs > totalRemainingMs) stepMs = totalRemainingMs;

    if (strafeRightNow) {
      if (inKickPhase) {
        strafeRight(PWM_STRAFE_KICK);
      } else {
        strafeRight(PWM_STRAFE);
      }
    } else {
      if (inKickPhase) {
        strafeLeft(PWM_STRAFE_KICK);
      } else {
        strafeLeft(PWM_STRAFE);
      }
    }

    delay(stepMs);
    elapsedMs += stepMs;

    if (watchColor) {
      sampleAccumMs += stepMs;
      if (sampleAccumMs >= COLOR_WATCH_SAMPLE_MS) {
        sampleAccumMs = 0UL;
        if (sampleColorWatchForStage1()) {
          stopMotors();
          delay(STRAFE_SETTLE_MS);
          return true;
        }
      }
    }
  }

  stopMotors();
  return false;
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

// Empirical distance-to-time tables for lateral correction.
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

bool runParallelStepShiftByDistance(bool strafeRightNow, int angleErrorDeg, float sensorWallDistCm, bool watchColor, const __FlashStringHelper *label) {
  setStatus("SHIFT", strafeRightNow ? "Move right" : "Move left");
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
  Serial.print(F(" watchColor="));
  Serial.print(watchColor ? F("true") : F("false"));
  Serial.print(F(" baseSum="));
  Serial.println(reading.sum);

  bool triggeredStage2 = executeTimedStrafeWithColorWatch(strafeRightNow, shiftMs, watchColor);
  delay(STRAFE_SETTLE_MS);
  return !triggeredStage2;
}

bool runParallelStepShift(bool strafeRightNow, int angleErrorDeg, unsigned long minStrafeMs, unsigned long maxStrafeMs, const __FlashStringHelper *label) {
  setStatus("SHIFT", strafeRightNow ? "Move right" : "Move left");
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
  return centerLightBySumPeakAtDistance(TARGET_WALL_DIST_CM, TARGET_DEPTH_TOL_CM, STOP_DIST_CM, true, STAGE1_LDR_WALL_DIST_CM, STAGE1_MIN_STRAFE_MS, STAGE1_MAX_STEP_SHIFT_MS, true);
}

bool centerLightBySumPeakAtDistance(float targetDistanceCm, float targetTolCm, float stopGuardCm, bool useDistanceModel, float sensorWallDistCm, unsigned long minStrafeMs, unsigned long maxStrafeMs, bool watchColorOnFirstStrafe) {
  int pass = 0;
  while (true) {
    ++pass;
    setStatus("SCAN", pass == 1 ? "Find light" : "Refine light");

    if (!alignPerpendicularWithStop(stopGuardCm, F("ALIGN"))) return false;
    if (!moveToTargetDistanceWithStop(targetDistanceCm, targetTolCm, stopGuardCm, F("DEPTH"))) return false;

    ScanResult scan;
    if (pass == 1) {
      scan = runScanAndLockRange(FULL_SCAN_MIN, FULL_SCAN_MAX, FULL_SCAN_STEP_DEG, FULL_SCAN_SCORE_TIE_MARGIN, F("SCAN_FULL"));
    } else {
      scan = runScanAndLockRange(NARROW_SCAN_MIN, NARROW_SCAN_MAX, NARROW_SCAN_STEP_DEG, NARROW_SCAN_SCORE_TIE_MARGIN, F("SCAN_NARROW"));
    }
    printScanResult(F("CENTER_SCAN"), scan);
    if (!scan.found) {
      setStatus("FAIL", "Light not found");
      return false;
    }

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
      setStatus("SCAN", "Light centered");
      Serial.println(F("[CENTER] Light is centered."));
      centerServo();
      return true;
    }

    bool strafeRightNow = POSITIVE_ANGLE_MEANS_STRAFE_RIGHT ? (angleError > 0) : (angleError < 0);
    if (useDistanceModel) {
      bool watchColor = watchColorOnFirstStrafe;
      if (!runParallelStepShiftByDistance(strafeRightNow, angleError, sensorWallDistCm, watchColor, F("PARALLEL_STEP"))) {
        if (stage1EarlyColorTrigger) {
          setStatus("COLOR", stage1LatchedColor == COLOR_GREEN ? "Found GREEN" : "Found RED");
          Serial.println(F("[CENTER] Early trigger during strafe."));
          return true;
        }
        setStatus("FAIL", "Shift failed");
        return false;
      }
    } else {
      if (!runParallelStepShift(strafeRightNow, angleError, minStrafeMs, maxStrafeMs, F("PARALLEL_STEP"))) return false;
    }
  }

  return false;
}

bool runAutoSequence() {
  Serial.println();
  Serial.println(F("========== AUTO START =========="));
  setStatus("AUTO", "Second stage");

  stage1EarlyColorTrigger = false;
  stage1LatchedColor = COLOR_ERROR;
  stage1ColorConfirmCount = 0;
  stage1ColorCandidate = COLOR_ERROR;
  centerServo();
  stopMotors();
  delay(300);

  if (!initialForwardUntilWallSeen()) return false;

  if (!centerLightBySumPeak()) return false;

  if (stage1EarlyColorTrigger) {
    Serial.println(F("[FINAL] Early trigger; jump to stage 2."));
  } else {
    Serial.println(F("[FINAL] Stage 1 alignment complete."));
  }

  if (!handlePostStageColor()) return false;

  Serial.println(F("[FINAL] Second stage complete."));
  centerServo();
  stopMotors();
  setStatus("DONE", "Sequence OK");
  showParkedScreen();
  return true;
}

// Debug output
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
  Serial.println(F("=== AMR Commands ==="));
  Serial.println(F("G : run auto sequence"));
  Serial.println(F("R : redo dark calibration"));
  Serial.println(F("S : run light scan only"));
  Serial.println(F("U : print ultrasonic pair"));
  Serial.println(F("Z : read color sensor once"));
  Serial.println(F("P : print current servo LDR reading"));
  Serial.println(F("C : center servo"));
  Serial.println(F("H or ? : help"));
  Serial.println();
}

// Serial commands
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
          ParkingReadings readings = readParkingReadings();
          printParkingReadings(F("ULTRA"), readings);
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

// Setup and loop
void setup() {
  Serial.begin(SERIAL_BAUD);
  initStatusMonitor();

  setupMotorPins();

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

  Serial.println(F("ELEC3848 AMR final sketch"));
  Serial.println(F("Flow: align -> color -> turn -> park."));
  Serial.println(F("Flip POSITIVE_ANGLE_MEANS_STRAFE_RIGHT if needed."));

  if (USE_PRESET_LDR_CALIBRATION) {
    loadPresetLdrCalibration();
  } else {
    doDarkCalibration();
  }
  printHelp();

  setStatus("BOOT", "Auto in 1 sec");
  Serial.println(F("[BOOT] Auto run starts in 1 second..."));
  delay(1000);
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
