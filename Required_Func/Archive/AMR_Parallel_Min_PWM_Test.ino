#include <Arduino.h>

// ============================================================
// Parallel minimum PWM finder
// ============================================================
// Purpose:
// 1. Find the minimum sideways strafe PWM that actually moves the robot.
// 2. Test left and right strafe independently.
// 3. Support coarse auto sweep and fine manual stepping.
//
// Important:
// - This sketch cannot detect motion by itself.
// - You must watch the robot and note the first PWM that really moves it.
// - Left and right thresholds may be different.

// ============================================================
// Pin Definitions
// ============================================================
const unsigned long SERIAL_BAUD = 115200UL;

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

const int PWM_MIN_LIMIT = 0;
const int PWM_MAX_LIMIT = 120;
const int PWM_START = 20;

const int COARSE_SWEEP_START = 16;
const int COARSE_SWEEP_END = 70;
const int COARSE_SWEEP_STEP = 2;

const unsigned long TEST_PULSE_MS = 350UL;
const unsigned long TEST_KICK_MS = 90UL;
const unsigned long TEST_SETTLE_MS = 1200UL;
const int TEST_KICK_EXTRA = 8;

// ============================================================
// Data Structures
// ============================================================
struct MotorPins
{
  uint8_t pwm;
  uint8_t in1;
  uint8_t in2;
};

// ============================================================
// Globals
// ============================================================
const MotorPins MOTORS[4] = {
  {M1_PWM, M1_IN1, M1_IN2},
  {M2_PWM, M2_IN1, M2_IN2},
  {M3_PWM, M3_IN1, M3_IN2},
  {M4_PWM, M4_IN1, M4_IN2}
};

int currentPwm = PWM_START;

// ============================================================
// Forward Declarations
// ============================================================
void setupMotorPins();
void writeMotorRaw(uint8_t pwmPin, uint8_t in1, uint8_t in2, int signedPwm);
void writeMotorByIndex(uint8_t index, int signedPwm);
void drive4(int pwmM1, int pwmM2, int pwmM3, int pwmM4);
void stopMotors();
void strafeLeft(int pwm);
void strafeRight(int pwm);
void runTimedStrafe(bool goRight, int pwm, unsigned long pulseMs);
void runSingleTest(bool goRight);
void runCoarseSweep(bool goRight);
void printStatus();
void printHelp();
void handleSerial();

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

void stopMotors()
{
  drive4(0, 0, 0, 0);
}

void strafeLeft(int pwm)
{
  drive4(-pwm, pwm, pwm, -pwm);
}

void strafeRight(int pwm)
{
  drive4(pwm, -pwm, -pwm, pwm);
}

// ============================================================
// Test Helpers
// ============================================================
void runTimedStrafe(bool goRight, int pwm, unsigned long pulseMs)
{
  int basePwm = constrain(pwm, PWM_MIN_LIMIT, PWM_MAX_LIMIT);
  int kickPwm = constrain(basePwm + TEST_KICK_EXTRA, PWM_MIN_LIMIT, PWM_MAX_LIMIT);

  Serial.print(F("[TEST] dir="));
  Serial.print(goRight ? F("RIGHT") : F("LEFT"));
  Serial.print(F(" pwm="));
  Serial.print(basePwm);
  Serial.print(F(" kickPwm="));
  Serial.print(kickPwm);
  Serial.print(F(" pulseMs="));
  Serial.println(pulseMs);

  unsigned long kickMs = (pulseMs < TEST_KICK_MS) ? pulseMs : TEST_KICK_MS;
  unsigned long holdMs = (pulseMs > kickMs) ? (pulseMs - kickMs) : 0UL;

  if (goRight)
  {
    strafeRight(kickPwm);
    delay(kickMs);
    if (holdMs > 0UL)
    {
      strafeRight(basePwm);
      delay(holdMs);
    }
  }
  else
  {
    strafeLeft(kickPwm);
    delay(kickMs);
    if (holdMs > 0UL)
    {
      strafeLeft(basePwm);
      delay(holdMs);
    }
  }

  stopMotors();
  Serial.println(F("[TEST] stop"));
  delay(TEST_SETTLE_MS);
}

void runSingleTest(bool goRight)
{
  runTimedStrafe(goRight, currentPwm, TEST_PULSE_MS);
}

void runCoarseSweep(bool goRight)
{
  Serial.println();
  Serial.print(F("[SWEEP] direction="));
  Serial.println(goRight ? F("RIGHT") : F("LEFT"));
  Serial.println(F("[SWEEP] Watch carefully and note the FIRST PWM where the robot actually starts moving."));

  for (int pwm = COARSE_SWEEP_START; pwm <= COARSE_SWEEP_END; pwm += COARSE_SWEEP_STEP)
  {
    runTimedStrafe(goRight, pwm, TEST_PULSE_MS);
  }

  Serial.println(F("[SWEEP] done"));
  Serial.println(F("[SWEEP] Then set current PWM near the first moving point and fine-tune by 1."));
  Serial.println();
}

void printStatus()
{
  Serial.print(F("[STATUS] currentPwm="));
  Serial.print(currentPwm);
  Serial.print(F(" pulseMs="));
  Serial.print(TEST_PULSE_MS);
  Serial.print(F(" kickMs="));
  Serial.print(TEST_KICK_MS);
  Serial.print(F(" kickExtra="));
  Serial.println(TEST_KICK_EXTRA);
}

void printHelp()
{
  Serial.println();
  Serial.println(F("=== Parallel Min PWM Test ==="));
  Serial.println(F("P : print current settings"));
  Serial.println(F("[ / ] : decrease / increase current PWM by 1"));
  Serial.println(F("{ / } : decrease / increase current PWM by 5"));
  Serial.println(F("L : test one LEFT strafe pulse at current PWM"));
  Serial.println(F("R : test one RIGHT strafe pulse at current PWM"));
  Serial.println(F("A : coarse LEFT sweep"));
  Serial.println(F("D : coarse RIGHT sweep"));
  Serial.println(F("S : stop motors"));
  Serial.println(F("H or ? : help"));
  Serial.println();
  Serial.println(F("Recommended method:"));
  Serial.println(F("1. Run A or D for a coarse sweep."));
  Serial.println(F("2. Note the first PWM that visibly moves."));
  Serial.println(F("3. Set current PWM near that point with [ ] or { }."));
  Serial.println(F("4. Use L or R repeatedly to find the exact threshold."));
  Serial.println();
}

void handleSerial()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') continue;

    switch (c)
    {
      case 'p':
      case 'P':
        printStatus();
        break;

      case '[':
        currentPwm = constrain(currentPwm - 1, PWM_MIN_LIMIT, PWM_MAX_LIMIT);
        printStatus();
        break;

      case ']':
        currentPwm = constrain(currentPwm + 1, PWM_MIN_LIMIT, PWM_MAX_LIMIT);
        printStatus();
        break;

      case '{':
        currentPwm = constrain(currentPwm - 5, PWM_MIN_LIMIT, PWM_MAX_LIMIT);
        printStatus();
        break;

      case '}':
        currentPwm = constrain(currentPwm + 5, PWM_MIN_LIMIT, PWM_MAX_LIMIT);
        printStatus();
        break;

      case 'l':
      case 'L':
        runSingleTest(false);
        break;

      case 'r':
      case 'R':
        runSingleTest(true);
        break;

      case 'a':
      case 'A':
        runCoarseSweep(false);
        break;

      case 'd':
      case 'D':
        runCoarseSweep(true);
        break;

      case 's':
      case 'S':
        stopMotors();
        Serial.println(F("[TEST] manual stop"));
        break;

      case 'h':
      case '?':
      case 'H':
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
void setup()
{
  Serial.begin(SERIAL_BAUD);
  setupMotorPins();
  stopMotors();

  Serial.println(F("ELEC3848 parallel minimum PWM finder"));
  Serial.println(F("This test checks sideways strafe threshold only."));
  Serial.println(F("Watch the robot and record the first PWM that truly moves it."));
  printHelp();
  printStatus();
}

void loop()
{
  handleSerial();
}
