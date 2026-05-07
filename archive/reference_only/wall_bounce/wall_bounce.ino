#include <Arduino.h>

// ============================================================
// Simple wall-bounce test
// Behavior:
// 1) Go forward
// 2) If front wall is near, stop
// 3) Turn 90 degrees
// 4) Repeat forever
//
// Motor movement logic is extracted from your tuned code.
// Only the navigation logic is simplified.
// ============================================================

// -----------------------------
// Serial
// -----------------------------
const unsigned long SERIAL_BAUD = 115200UL;

// -----------------------------
// Ultrasonic pins
// U1: left
// U2: right
// U3: back
// U4: front
// Assumption: first pin = TRIG, second pin = ECHO
// -----------------------------
const uint8_t US_LEFT_TRIG   = 33;  // U1 digital  = PC4 = D33
const uint8_t US_LEFT_ECHO   = 32;  // U1 analog   = PG1 = D32

const uint8_t US_RIGHT_TRIG  = 28;  // U2 digital  = PA6 = D28
const uint8_t US_RIGHT_ECHO  = 25;  // U2 analog   = PA3 = D25

const uint8_t US_REAR_TRIG   = 46;  // U3 digital  = PL3 = D46
const uint8_t US_REAR_ECHO   = 13;  // U3 analog   = PB7 = D13

const uint8_t US_FRONT_TRIG  = 30;  // U4 digital  = PC7 = D30
const uint8_t US_FRONT_ECHO  = 29;  // U4 analog   = PA7 = D29

// -----------------------------
// Motor pins
// Currently copied from your original style.
// Change to your actual motor pins if needed.
// -----------------------------
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

// -----------------------------
// Ultrasonic settings
// -----------------------------
const unsigned long ULTRA_TIMEOUT_US = 25000UL;
const float ULTRA_VALID_MIN_CM = 2.0f;
const float ULTRA_VALID_MAX_CM = 400.0f;

// Distance threshold for wall detection
const float FRONT_WALL_STOP_CM = 25.0f;

// -----------------------------
// Movement tuning
// Extracted from your original code
// -----------------------------
const int MOTOR_SIGN[4] = { -1, +1, -1, +1 };
const int MOTOR_TRIM[4] = { 0, 0, 0, 0 };
int motorRuntimeTrim[4] = { 0, 0, 0, 0 };

const int SIDE_BALANCE = 0;

// Tuned motion PWM values from your original code
const int PWM_FORWARD = 42;   // from PWM_APPROACH_FAST
const int PWM_BACKUP  = 34;
const int PWM_TURN_90 = 28;

// Tuned motor scaling ratios from your original code
const float FORWARD_RATIO[4]  = { 0.9650f, 0.8462f, 0.9650f, 0.8462f };
const float BACKWARD_RATIO[4] = { 1.0000f, 0.8462f, 1.0000f, 0.8462f };

// 90-degree timing from your original code
const unsigned long TURN_LEFT_90_MS  = 2400UL;
const unsigned long TURN_RIGHT_90_MS = 2550UL;

// small settle delays
const unsigned long STOP_SETTLE_MS = 150UL;
const unsigned long LOOP_DELAY_MS  = 30UL;

// -----------------------------
// Motor structure
// -----------------------------
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
// Utility
// ============================================================
bool isDistanceValid(float cm) {
  return (cm >= ULTRA_VALID_MIN_CM && cm <= ULTRA_VALID_MAX_CM);
}

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

float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

float readDistanceMedianCm(uint8_t trigPin, uint8_t echoPin) {
  float d1 = readDistanceCm(trigPin, echoPin);
  delay(8);
  float d2 = readDistanceCm(trigPin, echoPin);
  delay(8);
  float d3 = readDistanceCm(trigPin, echoPin);
  return median3(d1, d2, d3);
}

// ============================================================
// Motor control (extracted / simplified from your original)
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
  int leftAdjusted  = leftPwm  - SIDE_BALANCE;
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
    scaleMotionPwm(pwm, FORWARD_RATIO[3])
  );
}

void driveBackward(int pwm) {
  drive4(
    -scaleMotionPwm(pwm, BACKWARD_RATIO[0]),
    -scaleMotionPwm(pwm, BACKWARD_RATIO[1]),
    -scaleMotionPwm(pwm, BACKWARD_RATIO[2]),
    -scaleMotionPwm(pwm, BACKWARD_RATIO[3])
  );
}

void rotateLeft(int pwm) {
  driveTank(pwm, -pwm);
}

void rotateRight(int pwm) {
  driveTank(-pwm, pwm);
}

// ============================================================
// Simple navigation
// ============================================================
void setupUltrasonicPins() {
  pinMode(US_FRONT_TRIG, OUTPUT);
  pinMode(US_FRONT_ECHO, INPUT);

  pinMode(US_LEFT_TRIG, OUTPUT);
  pinMode(US_LEFT_ECHO, INPUT);

  pinMode(US_RIGHT_TRIG, OUTPUT);
  pinMode(US_RIGHT_ECHO, INPUT);

  pinMode(US_REAR_TRIG, OUTPUT);
  pinMode(US_REAR_ECHO, INPUT);

  digitalWrite(US_FRONT_TRIG, LOW);
  digitalWrite(US_LEFT_TRIG, LOW);
  digitalWrite(US_RIGHT_TRIG, LOW);
  digitalWrite(US_REAR_TRIG, LOW);
}

void rotateNinetyDegreesRight() {
  Serial.println(F("[TURN] RIGHT 90"));
  rotateRight(PWM_TURN_90);
  delay(TURN_RIGHT_90_MS);
  stopMotors();
  delay(STOP_SETTLE_MS);
}

void rotateNinetyDegreesLeft() {
  Serial.println(F("[TURN] LEFT 90"));
  rotateLeft(PWM_TURN_90);
  delay(TURN_LEFT_90_MS);
  stopMotors();
  delay(STOP_SETTLE_MS);
}

void goForwardUntilWallThenTurn() {
  float frontCm = readDistanceMedianCm(US_FRONT_TRIG, US_FRONT_ECHO);

  Serial.print(F("[FRONT] "));
  Serial.print(frontCm, 1);
  Serial.println(F(" cm"));

  if (frontCm <= FRONT_WALL_STOP_CM) {
    stopMotors();
    delay(STOP_SETTLE_MS);

    // Optional small backup if too close
    if (frontCm < 12.0f) {
      Serial.println(F("[ACTION] BACKUP"));
      driveBackward(PWM_BACKUP);
      delay(180);
      stopMotors();
      delay(STOP_SETTLE_MS);
    }

    Serial.println(F("[ACTION] TURN RIGHT"));
    rotateNinetyDegreesRight();
  } else {
    Serial.println(F("[ACTION] FORWARD"));
    driveForward(PWM_FORWARD);
  }
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  setupMotorPins();
  setupUltrasonicPins();

  stopMotors();
  delay(500);

  Serial.println(F("Simple wall-bounce robot start"));
  Serial.println(F("Behavior: forward -> wall near -> 90deg turn -> repeat"));
}

void loop() {
  goForwardUntilWallThenTurn();
  delay(LOOP_DELAY_MS);
}