#include <Arduino.h>

/*
Timed 90-degree turn test sketch

Purpose:
- isolate the exact timed 90-degree turn behavior currently used in AMR_Final_V3
- no ultrasonic, servo, color, OLED, or parking logic

Serial commands at 115200:
- r : turn right 90 degrees
- l : turn left 90 degrees
- s : drive straight for 4000 ms
- x : stop motors
- p : print active settings
- h or ? : help
*/

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
// Turn Stack copied from AMR_Final_V3
// ============================================================
const int MOTOR_SIGN[4] = { -1, +1, -1, +1 };
const int MOTOR_TRIM[4] = { 0, 0, 0, 0 };
const int SIDE_BALANCE = 0;

const int PWM_STRAIGHT_TEST = 40;
const int PWM_TURN_90 = 28;
const unsigned long STRAIGHT_TEST_MS = 4000UL;
const unsigned long TURN_LEFT_90_MS = 2640UL;
const unsigned long TURN_RIGHT_90_MS = 2550UL;
const unsigned long STRAFE_SETTLE_MS = 150UL;

const float FORWARD_RATIO[4] = { 1.0000f, 0.8462f, 1.0000f, 0.8462f };

// ============================================================
// Data Structures
// ============================================================
struct MotorPins {
  uint8_t pwm;
  uint8_t in1;
  uint8_t in2;
};

const MotorPins MOTORS[4] = {
  { M1_PWM, M1_IN1, M1_IN2 },
  { M2_PWM, M2_IN1, M2_IN2 },
  { M3_PWM, M3_IN1, M3_IN2 },
  { M4_PWM, M4_IN1, M4_IN2 }
};

// ============================================================
// Motor Layer copied from AMR_Final_V3
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

void rotateLeft(int pwm) {
  driveTank(pwm, -pwm);
}

void rotateRight(int pwm) {
  driveTank(-pwm, pwm);
}

void driveForward(int pwm) {
  drive4(
    scaleMotionPwm(pwm, FORWARD_RATIO[0]),
    scaleMotionPwm(pwm, FORWARD_RATIO[1]),
    scaleMotionPwm(pwm, FORWARD_RATIO[2]),
    scaleMotionPwm(pwm, FORWARD_RATIO[3]));
}

// ============================================================
// Timed 90-degree turn copied from AMR_Final_V3
// ============================================================
void runStraightTest() {
  Serial.print(F("[STRAIGHT] pwm="));
  Serial.print(PWM_STRAIGHT_TEST);
  Serial.print(F(" durationMs="));
  Serial.print(STRAIGHT_TEST_MS);
  Serial.print(F(" settleMs="));
  Serial.println(STRAFE_SETTLE_MS);

  driveForward(PWM_STRAIGHT_TEST);
  delay(STRAIGHT_TEST_MS);
  stopMotors();
  delay(STRAFE_SETTLE_MS);
  Serial.println(F("[STRAIGHT] done"));
}

void rotateNinetyDegrees(bool clockwise) {
  Serial.print(F("[TURN] 90 deg "));
  Serial.println(clockwise ? F("CLOCKWISE") : F("ANTI_CLOCKWISE"));

  if (clockwise) {
    Serial.print(F("[TURN] pwm="));
    Serial.print(PWM_TURN_90);
    Serial.print(F(" durationMs="));
    Serial.print(TURN_RIGHT_90_MS);
    Serial.print(F(" settleMs="));
    Serial.println(STRAFE_SETTLE_MS);
    rotateRight(PWM_TURN_90);
    delay(TURN_RIGHT_90_MS);
  } else {
    Serial.print(F("[TURN] pwm="));
    Serial.print(PWM_TURN_90);
    Serial.print(F(" durationMs="));
    Serial.print(TURN_LEFT_90_MS);
    Serial.print(F(" settleMs="));
    Serial.println(STRAFE_SETTLE_MS);
    rotateLeft(PWM_TURN_90);
    delay(TURN_LEFT_90_MS);
  }

  stopMotors();
  delay(STRAFE_SETTLE_MS);
  Serial.println(F("[TURN] done"));
}

// ============================================================
// Serial UI
// ============================================================
void printSettings() {
  Serial.println();
  Serial.println(F("=== Timed Turn Settings ==="));
  Serial.print(F("PWM_STRAIGHT_TEST = "));
  Serial.println(PWM_STRAIGHT_TEST);
  Serial.print(F("STRAIGHT_TEST_MS = "));
  Serial.println(STRAIGHT_TEST_MS);
  Serial.print(F("PWM_TURN_90 = "));
  Serial.println(PWM_TURN_90);
  Serial.print(F("TURN_LEFT_90_MS = "));
  Serial.println(TURN_LEFT_90_MS);
  Serial.print(F("TURN_RIGHT_90_MS = "));
  Serial.println(TURN_RIGHT_90_MS);
  Serial.print(F("STRAFE_SETTLE_MS = "));
  Serial.println(STRAFE_SETTLE_MS);

  Serial.print(F("MOTOR_SIGN = {"));
  Serial.print(MOTOR_SIGN[0]); Serial.print(F(", "));
  Serial.print(MOTOR_SIGN[1]); Serial.print(F(", "));
  Serial.print(MOTOR_SIGN[2]); Serial.print(F(", "));
  Serial.print(MOTOR_SIGN[3]); Serial.println(F("}"));

  Serial.print(F("MOTOR_TRIM = {"));
  Serial.print(MOTOR_TRIM[0]); Serial.print(F(", "));
  Serial.print(MOTOR_TRIM[1]); Serial.print(F(", "));
  Serial.print(MOTOR_TRIM[2]); Serial.print(F(", "));
  Serial.print(MOTOR_TRIM[3]); Serial.println(F("}"));

  Serial.print(F("SIDE_BALANCE = "));
  Serial.println(SIDE_BALANCE);

  Serial.println(F("Wheel mapping:"));
  Serial.println(F("M1 = left front"));
  Serial.println(F("M2 = right front"));
  Serial.println(F("M3 = left rear"));
  Serial.println(F("M4 = right rear"));
  Serial.println();
}

void printHelp() {
  Serial.println();
  Serial.println(F("=== Timed 90-Degree Turn Test ==="));
  Serial.println(F("r : rotate right 90 degrees"));
  Serial.println(F("l : rotate left 90 degrees"));
  Serial.println(F("s : drive straight for 4000 ms"));
  Serial.println(F("x : stop motors"));
  Serial.println(F("p : print current settings"));
  Serial.println(F("h or ? : help"));
  Serial.println();
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') continue;
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');

    switch (c) {
      case 'R':
        rotateNinetyDegrees(true);
        break;

      case 'L':
        rotateNinetyDegrees(false);
        break;

      case 'S':
        runStraightTest();
        break;

      case 'X':
        stopMotors();
        Serial.println(F("[CMD] STOP"));
        break;

      case 'P':
        printSettings();
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

  Serial.println(F("AMR timed 90-degree turn test"));
  Serial.println(F("Uses the exact timed turn behavior from AMR_Final_V3."));
  printHelp();
  printSettings();
}

void loop() {
  handleSerial();
}
