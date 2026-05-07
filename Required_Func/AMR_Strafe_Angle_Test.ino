#include <Arduino.h>
#include <Servo.h>

/*
Strafe + angle calibration sketch

Flow:
1. Send `g` in Serial Monitor.
2. Robot moves forward until one front ultrasonic sees the wall at about 35 cm.
3. Robot aligns perpendicular to the wall.
4. Robot approaches until average wall distance is about 33 cm.
5. Use commands:
   - `r100` -> strafe right for 100 ms
   - `l250` -> strafe left for 250 ms
   - `a`    -> run light-angle scan from 55..125 and print the best angle

This sketch uses the current stage-1 strafe actuation force:
- base PWM = 34
- kick PWM = 42

The angle scan uses the stage-2 light-selection logic:
- preset LDR calibration
- dark-subtracted sum
- shortlist within tie margin of the global best
- choose the shortlist sample closest to center, then smaller balance diff
*/

// ============================================================
// Pin Definitions
// ============================================================
const unsigned long SERIAL_BAUD = 115200UL;

const uint8_t SERVO_PIN = 28;
const uint8_t LDR_LEFT_PIN  = A0;
const uint8_t LDR_RIGHT_PIN = A2;

const uint8_t US_L_TRIG = 32;
const uint8_t US_L_ECHO = 33;
const uint8_t US_R_TRIG = 22;
const uint8_t US_R_ECHO = 24;

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
const int MOTOR_SIGN[4] = {-1, +1, -1, +1};
const int MOTOR_TRIM[4] = {0, -8, 0, -8};
const int SIDE_BALANCE = 0;

const int PWM_INITIAL_FORWARD = 50;
const int PWM_APPROACH_FAST = 42;
const int PWM_APPROACH_SLOW = 30;
const int PWM_BACKUP = 34;
const int PWM_STRAFE = 34;
const int PWM_STRAFE_KICK = 42;
const int PWM_ALIGN_BASE = 24;
const int PWM_ALIGN_EXTRA = 12;

const unsigned long INITIAL_FORWARD_TIMEOUT_MS = 12000UL;
const unsigned long INITIAL_FORWARD_LOOP_DELAY_MS = 25UL;
const unsigned long ALIGN_LOOP_DELAY_MS = 20UL;
const unsigned long APPROACH_BURST_MS = 140UL;
const unsigned long BACKUP_BURST_MS = 100UL;
const unsigned long STRAFE_KICK_MS = 90UL;
const unsigned long STRAFE_SETTLE_MS = 150UL;

const float INITIAL_FORWARD_DETECT_CM = 35.0f;
const float TARGET_WALL_DIST_CM = 33.0f;
const float TARGET_DEPTH_TOL_CM = 1.5f;
const float PERP_TOL_CM = 1.5f;
const float STOP_DIST_CM = 24.0f;
const float SLOW_DIST_CM = 38.0f;

const int ULTRA_TIMEOUT_US = 25000;
const int ULTRA_VALID_MIN_CM = 2;
const int ULTRA_VALID_MAX_CM = 400;
const int ALIGN_RETRIES = 40;
const int APPROACH_RETRIES = 40;
const int ALIGN_STABLE_COUNT = 2;

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
const int ANGLE_TEST_SCAN_MIN = 55;
const int ANGLE_TEST_SCAN_MAX = 125;
const int ANGLE_TEST_SCAN_STEP_DEG = 2;
const int SCAN_SETTLE_MS = 120;
const int MIN_SCAN_SCORE = 20;
const int ANGLE_TEST_TIE_MARGIN = 3;

// ============================================================
// Data Structures
// ============================================================
struct MotorPins
{
  uint8_t pwm;
  uint8_t in1;
  uint8_t in2;
};

struct UltraPair
{
  bool leftValid;
  bool rightValid;
  bool valid;
  float leftCm;
  float rightCm;
  float avgCm;
  float diffCm;
};

struct LdrReading
{
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

struct ScanResult
{
  bool found;
  int bestAngle;
  int bestScore;
  LdrReading bestReading;
};

// ============================================================
// Globals
// ============================================================
Servo trackerServo;

const MotorPins MOTORS[4] = {
  {M1_PWM, M1_IN1, M1_IN2},
  {M2_PWM, M2_IN1, M2_IN2},
  {M3_PWM, M3_IN1, M3_IN2},
  {M4_PWM, M4_IN1, M4_IN2}
};

bool preparedAtWall = false;
int servoPos = SERVO_CENTER;
int darkA = PRESET_DARK_A;
int darkB = PRESET_DARK_B;
int ambientA = PRESET_AMBIENT_A;
int ambientB = PRESET_AMBIENT_B;
float gainA = PRESET_GAIN_A;
float gainB = PRESET_GAIN_B;
float ldrTrim = PRESET_LDR_TRIM;

// ============================================================
// Helpers
// ============================================================
float floatAbs(float value)
{
  return (value < 0.0f) ? -value : value;
}

float clampFloat(float value, float minValue, float maxValue)
{
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

bool isDistanceValid(float cm)
{
  return (cm >= (float)ULTRA_VALID_MIN_CM && cm <= (float)ULTRA_VALID_MAX_CM);
}

float median3(float a, float b, float c)
{
  if (a > b)
  {
    float t = a; a = b; b = t;
  }
  if (b > c)
  {
    float t = b; b = c; c = t;
  }
  if (a > b)
  {
    float t = a; a = b; b = t;
  }
  return b;
}

// ============================================================
// Motor Layer
// ============================================================
void setupMotorPins()
{
  pinMode(M1_PWM, OUTPUT); pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT); pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);
  pinMode(M4_PWM, OUTPUT); pinMode(M4_IN1, OUTPUT); pinMode(M4_IN2, OUTPUT);
}

void writeMotorRaw(uint8_t pwmPin, uint8_t in1, uint8_t in2, int signedPwm)
{
  signedPwm = constrain(signedPwm, -255, 255);

  if (signedPwm > 0)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, signedPwm);
  }
  else if (signedPwm < 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -signedPwm);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void writeMotorByIndex(uint8_t index, int signedPwm)
{
  if (signedPwm != 0)
  {
    int direction = (signedPwm > 0) ? 1 : -1;
    int magnitude = abs(signedPwm);
    magnitude = constrain(magnitude + MOTOR_TRIM[index], 0, 255);
    signedPwm = direction * magnitude;
  }

  signedPwm *= MOTOR_SIGN[index];
  writeMotorRaw(MOTORS[index].pwm, MOTORS[index].in1, MOTORS[index].in2, signedPwm);
}

void drive4(int pwmM1, int pwmM2, int pwmM3, int pwmM4)
{
  writeMotorByIndex(0, pwmM1);
  writeMotorByIndex(1, pwmM2);
  writeMotorByIndex(2, pwmM3);
  writeMotorByIndex(3, pwmM4);
}

void driveTank(int leftPwm, int rightPwm)
{
  int leftAdjusted = leftPwm - SIDE_BALANCE;
  int rightAdjusted = rightPwm + SIDE_BALANCE;
  drive4(leftAdjusted, rightAdjusted, leftAdjusted, rightAdjusted);
}

void stopMotors()
{
  drive4(0, 0, 0, 0);
}

void driveForward(int pwm)
{
  driveTank(pwm, pwm);
}

void driveBackward(int pwm)
{
  driveTank(-pwm, -pwm);
}

void rotateLeft(int pwm)
{
  driveTank(pwm, -pwm);
}

void rotateRight(int pwm)
{
  driveTank(-pwm, pwm);
}

void strafeLeft(int pwm)
{
  drive4(-pwm, pwm, pwm, -pwm);
}

void strafeRight(int pwm)
{
  drive4(pwm, -pwm, -pwm, pwm);
}

void executeTimedStrafe(bool strafeRightNow, unsigned long totalMs)
{
  unsigned long kickMs = (totalMs < STRAFE_KICK_MS) ? totalMs : STRAFE_KICK_MS;
  unsigned long holdMs = (totalMs > kickMs) ? (totalMs - kickMs) : 0UL;

  if (strafeRightNow)
  {
    strafeRight(PWM_STRAFE_KICK);
    delay(kickMs);
    if (holdMs > 0UL)
    {
      strafeRight(PWM_STRAFE);
      delay(holdMs);
    }
  }
  else
  {
    strafeLeft(PWM_STRAFE_KICK);
    delay(kickMs);
    if (holdMs > 0UL)
    {
      strafeLeft(PWM_STRAFE);
      delay(holdMs);
    }
  }

  stopMotors();
  delay(STRAFE_SETTLE_MS);
}

// ============================================================
// Servo / LDR Layer
// ============================================================
void writeServoAngle(int angle)
{
  servoPos = constrain(angle, SERVO_MIN, SERVO_MAX);
  trackerServo.write(servoPos);
}

void centerServo()
{
  writeServoAngle(SERVO_CENTER);
}

int readAverage(int pin, int samples = LDR_SAMPLES)
{
  long sum = 0;
  for (int i = 0; i < samples; ++i)
  {
    sum += analogRead(pin);
    delay(LDR_SAMPLE_DELAY_MS);
  }
  return (int)(sum / samples);
}

float normalizeLdr(int rawValue, int darkValue, int ambientValue)
{
  int span = ambientValue - darkValue;
  if (span < 1) span = 1;

  float normalized = 100.0f * (float)(rawValue - darkValue) / (float)span;
  return clampFloat(normalized, 0.0f, LDR_NORM_MAX);
}

LdrReading readLdrAtAngle(int angle)
{
  LdrReading reading;

  writeServoAngle(angle);
  delay(SCAN_SETTLE_MS);

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

void printLdrReading(const __FlashStringHelper *label, const LdrReading &reading)
{
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

ScanResult runAngleTestScan()
{
  ScanResult result;
  result.found = false;
  result.bestAngle = SERVO_CENTER;
  result.bestScore = -1;

  const int maxSamples = ((ANGLE_TEST_SCAN_MAX - ANGLE_TEST_SCAN_MIN) / ANGLE_TEST_SCAN_STEP_DEG) + 2;
  LdrReading readings[maxSamples];
  int readingCount = 0;
  int globalBestScore = -1;

  Serial.println(F("[ANGLE] ===== Start angle scan ====="));
  Serial.print(F("[ANGLE] range="));
  Serial.print(ANGLE_TEST_SCAN_MIN);
  Serial.print(F(".."));
  Serial.print(ANGLE_TEST_SCAN_MAX);
  Serial.print(F(" step="));
  Serial.println(ANGLE_TEST_SCAN_STEP_DEG);

  for (int angle = ANGLE_TEST_SCAN_MIN; angle <= ANGLE_TEST_SCAN_MAX; angle += ANGLE_TEST_SCAN_STEP_DEG)
  {
    LdrReading reading = readLdrAtAngle(angle);
    printLdrReading(F("ANGLE_SCAN"), reading);

    if (readingCount < maxSamples)
    {
      readings[readingCount++] = reading;
    }

    if (reading.sum > globalBestScore)
    {
      globalBestScore = reading.sum;
    }
  }

  if (readingCount == 0 || globalBestScore < MIN_SCAN_SCORE)
  {
    Serial.println(F("[ANGLE] No valid light peak found."));
    centerServo();
    return result;
  }

  for (int i = 0; i < readingCount; ++i)
  {
    const LdrReading &reading = readings[i];
    if (globalBestScore - reading.sum > ANGLE_TEST_TIE_MARGIN) continue;

    bool better = false;
    if (!result.found)
    {
      better = true;
    }
    else
    {
      int currentOffset = abs(reading.pan - SERVO_CENTER);
      int bestOffset = abs(result.bestAngle - SERVO_CENTER);
      float currentDiffAbs = floatAbs(reading.diff);
      float bestDiffAbs = floatAbs(result.bestReading.diff);

      if (currentOffset < bestOffset)
      {
        better = true;
      }
      else if (currentOffset == bestOffset && currentDiffAbs < bestDiffAbs - 0.5f)
      {
        better = true;
      }
      else if (currentOffset == bestOffset && currentDiffAbs <= bestDiffAbs + 0.5f && reading.sum > result.bestScore)
      {
        better = true;
      }
    }

    if (better)
    {
      result.found = true;
      result.bestAngle = reading.pan;
      result.bestScore = reading.sum;
      result.bestReading = reading;
    }
  }

  if (!result.found)
  {
    Serial.println(F("[ANGLE] No shortlist candidate found."));
    centerServo();
    return result;
  }

  writeServoAngle(result.bestAngle);
  delay(SCAN_SETTLE_MS);
  result.bestReading = readLdrAtAngle(result.bestAngle);

  Serial.print(F("[ANGLE] Best angle="));
  Serial.print(result.bestAngle);
  Serial.print(F(" bestScore="));
  Serial.print(result.bestScore);
  Serial.print(F(" angleError="));
  Serial.println(result.bestAngle - SERVO_CENTER);
  return result;
}

// ============================================================
// Ultrasonic Layer
// ============================================================
float readDistanceCm(uint8_t trigPin, uint8_t echoPin)
{
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

UltraPair readUltrasonicPair()
{
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

  if (pair.valid)
  {
    pair.avgCm = 0.5f * (pair.leftCm + pair.rightCm);
    pair.diffCm = pair.leftCm - pair.rightCm;
  }
  else if (pair.leftValid)
  {
    pair.avgCm = pair.leftCm;
    pair.diffCm = 0.0f;
  }
  else if (pair.rightValid)
  {
    pair.avgCm = pair.rightCm;
    pair.diffCm = 0.0f;
  }
  else
  {
    pair.avgCm = 999.0f;
    pair.diffCm = 0.0f;
  }

  return pair;
}

void printUltraPair(const __FlashStringHelper *label, const UltraPair &pair)
{
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

// ============================================================
// Control Phases
// ============================================================
bool initialForwardUntilWallSeen()
{
  Serial.println(F("[INITIAL] Start forward-until-first-35cm detection"));

  unsigned long startMs = millis();
  driveForward(PWM_INITIAL_FORWARD);

  while ((millis() - startMs) <= INITIAL_FORWARD_TIMEOUT_MS)
  {
    UltraPair pair = readUltrasonicPair();
    printUltraPair(F("INITIAL"), pair);

    bool leftHit = pair.leftValid && pair.leftCm <= INITIAL_FORWARD_DETECT_CM;
    bool rightHit = pair.rightValid && pair.rightCm <= INITIAL_FORWARD_DETECT_CM;

    if ((pair.leftValid && pair.leftCm <= STOP_DIST_CM) || (pair.rightValid && pair.rightCm <= STOP_DIST_CM))
    {
      stopMotors();
      Serial.println(F("[INITIAL] Stop-distance guard hit."));
      return false;
    }

    if (leftHit || rightHit)
    {
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

bool alignPerpendicular()
{
  Serial.println(F("[ALIGN] Start perpendicular alignment"));
  int stableCount = 0;

  for (int attempt = 0; attempt < ALIGN_RETRIES; ++attempt)
  {
    UltraPair pair = readUltrasonicPair();
    printUltraPair(F("ALIGN"), pair);

    if (!pair.valid)
    {
      stableCount = 0;

      if ((pair.leftValid && pair.leftCm <= STOP_DIST_CM) || (pair.rightValid && pair.rightCm <= STOP_DIST_CM))
      {
        stopMotors();
        Serial.println(F("[ALIGN] One valid ultrasonic is too close to wall, abort."));
        return false;
      }

      if (pair.leftValid && !pair.rightValid)
      {
        rotateLeft(PWM_ALIGN_BASE + PWM_ALIGN_EXTRA);
        delay(ALIGN_LOOP_DELAY_MS);
        continue;
      }

      if (pair.rightValid && !pair.leftValid)
      {
        rotateRight(PWM_ALIGN_BASE + PWM_ALIGN_EXTRA);
        delay(ALIGN_LOOP_DELAY_MS);
        continue;
      }

      stopMotors();
      delay(STRAFE_SETTLE_MS);
      continue;
    }

    if (pair.avgCm <= STOP_DIST_CM)
    {
      stopMotors();
      Serial.println(F("[ALIGN] Too close to wall, abort."));
      return false;
    }

    if (floatAbs(pair.diffCm) <= PERP_TOL_CM)
    {
      stableCount++;
      if (stableCount >= ALIGN_STABLE_COUNT)
      {
        stopMotors();
        Serial.println(F("[ALIGN] Perpendicular OK"));
        return true;
      }

      delay(ALIGN_LOOP_DELAY_MS);
      continue;
    }

    stableCount = 0;
    float diffAbs = floatAbs(pair.diffCm);
    int pwm = PWM_ALIGN_BASE;

    if (diffAbs >= 8.0f)
    {
      pwm += PWM_ALIGN_EXTRA;
    }
    else if (diffAbs >= 3.0f)
    {
      pwm += PWM_ALIGN_EXTRA / 2;
    }

    if (pair.diffCm > 0.0f)
    {
      rotateLeft(pwm);
    }
    else
    {
      rotateRight(pwm);
    }

    delay(ALIGN_LOOP_DELAY_MS);
  }

  stopMotors();
  Serial.println(F("[ALIGN] Failed to converge."));
  return false;
}

bool moveToTargetDistance()
{
  Serial.println(F("[DEPTH] Start approach to 33 cm"));

  for (int attempt = 0; attempt < APPROACH_RETRIES; ++attempt)
  {
    UltraPair pair = readUltrasonicPair();
    printUltraPair(F("DEPTH"), pair);

    if (!pair.valid)
    {
      stopMotors();
      delay(STRAFE_SETTLE_MS);
      continue;
    }

    if (floatAbs(pair.diffCm) > PERP_TOL_CM)
    {
      stopMotors();
      if (!alignPerpendicular()) return false;
      continue;
    }

    if (pair.avgCm <= STOP_DIST_CM)
    {
      stopMotors();
      Serial.println(F("[DEPTH] Hit stop-distance guard."));
      return false;
    }

    float depthError = pair.avgCm - TARGET_WALL_DIST_CM;
    if (floatAbs(depthError) <= TARGET_DEPTH_TOL_CM)
    {
      stopMotors();
      Serial.println(F("[DEPTH] Target distance reached."));
      return true;
    }

    if (depthError > 0.0f)
    {
      int pwm = (pair.avgCm <= SLOW_DIST_CM) ? PWM_APPROACH_SLOW : PWM_APPROACH_FAST;
      driveForward(pwm);
      delay(APPROACH_BURST_MS);
    }
    else
    {
      driveBackward(PWM_BACKUP);
      delay(BACKUP_BURST_MS);
    }

    stopMotors();
    delay(STRAFE_SETTLE_MS);
  }

  stopMotors();
  Serial.println(F("[DEPTH] Failed to reach target distance."));
  return false;
}

bool prepareAtWall()
{
  preparedAtWall = false;

  if (!initialForwardUntilWallSeen()) return false;
  if (!alignPerpendicular()) return false;
  if (!moveToTargetDistance()) return false;

  preparedAtWall = true;
  Serial.println(F("[READY] Wall prep complete. Enter l### / r### / a."));
  printUltraPair(F("READY"), readUltrasonicPair());
  return true;
}

// ============================================================
// Serial Commands
// ============================================================
void printHelp()
{
  Serial.println();
  Serial.println(F("=== Strafe Angle Test Commands ==="));
  Serial.println(F("G    : go to wall, align perpendicular, approach to 33 cm"));
  Serial.println(F("R100 : strafe right for 100 ms"));
  Serial.println(F("L250 : strafe left for 250 ms"));
  Serial.println(F("A    : run 55..125 light-angle scan and print best angle"));
  Serial.println(F("N    : print current LDR reading at current servo angle"));
  Serial.println(F("C    : center servo"));
  Serial.println(F("P    : print current ultrasonic readings"));
  Serial.println(F("S    : stop motors"));
  Serial.println(F("H/?  : help"));
  Serial.println();
}

void handleCommand(const char *command)
{
  if (command[0] == '\0') return;

  char c = command[0];
  if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');

  if (c == 'H' || c == '?')
  {
    printHelp();
    return;
  }

  if (c == 'G')
  {
    stopMotors();
    prepareAtWall();
    return;
  }

  if (c == 'P')
  {
    printUltraPair(F("STATUS"), readUltrasonicPair());
    return;
  }

  if (c == 'C')
  {
    centerServo();
    Serial.println(F("[SERVO] Centered."));
    return;
  }

  if (c == 'N')
  {
    printLdrReading(F("NOW"), readLdrAtAngle(servoPos));
    return;
  }

  if (c == 'A')
  {
    ScanResult scan = runAngleTestScan();
    if (!scan.found)
    {
      Serial.println(F("[ANGLE] Scan failed."));
    }
    return;
  }

  if (c == 'S' || c == 'X')
  {
    stopMotors();
    Serial.println(F("[STOP] Motors stopped."));
    return;
  }

  if (c == 'L' || c == 'R')
  {
    unsigned long strafeMs = strtoul(command + 1, NULL, 10);
    if (strafeMs == 0UL)
    {
      Serial.println(F("[CMD] Invalid strafe time. Use l### or r###."));
      return;
    }

    Serial.print(F("[CMD] dir="));
    Serial.print(c == 'R' ? F("RIGHT") : F("LEFT"));
    Serial.print(F(" totalMs="));
    Serial.print(strafeMs);
    Serial.print(F(" basePwm="));
    Serial.print(PWM_STRAFE);
    Serial.print(F(" kickPwm="));
    Serial.print(PWM_STRAFE_KICK);
    Serial.print(F(" prepared="));
    Serial.println(preparedAtWall ? F("true") : F("false"));

    executeTimedStrafe(c == 'R', strafeMs);
    printUltraPair(F("POST_STRAFE"), readUltrasonicPair());
    return;
  }

  Serial.print(F("[CMD] Unknown command: "));
  Serial.println(command);
  printHelp();
}

void handleSerial()
{
  static char buffer[32];
  static uint8_t length = 0;

  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n')
    {
      buffer[length] = '\0';
      handleCommand(buffer);
      length = 0;
      buffer[0] = '\0';
      continue;
    }

    if (length < sizeof(buffer) - 1)
    {
      buffer[length++] = c;
    }
  }
}

// ============================================================
// Setup / Loop
// ============================================================
void setup()
{
  Serial.begin(SERIAL_BAUD);

  setupMotorPins();

  pinMode(US_L_TRIG, OUTPUT);
  pinMode(US_L_ECHO, INPUT);
  pinMode(US_R_TRIG, OUTPUT);
  pinMode(US_R_ECHO, INPUT);
  pinMode(LDR_LEFT_PIN, INPUT);
  pinMode(LDR_RIGHT_PIN, INPUT);

  digitalWrite(US_L_TRIG, LOW);
  digitalWrite(US_R_TRIG, LOW);

  trackerServo.attach(SERVO_PIN);
  centerServo();
  stopMotors();

  Serial.println(F("AMR strafe + angle test sketch"));
  Serial.print(F("[CAL] preset darkA="));
  Serial.print(darkA);
  Serial.print(F(" darkB="));
  Serial.print(darkB);
  Serial.print(F(" ambientA="));
  Serial.print(ambientA);
  Serial.print(F(" ambientB="));
  Serial.println(ambientB);
  Serial.print(F("Uses fixed strafe actuation: basePwm="));
  Serial.print(PWM_STRAFE);
  Serial.print(F(" kickPwm="));
  Serial.println(PWM_STRAFE_KICK);
  printHelp();
}

void loop()
{
  handleSerial();
}
