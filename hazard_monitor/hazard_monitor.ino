#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ctype.h>
#include <string.h>

// ============================================================
// Basic config
// ============================================================
const unsigned long SERIAL_BAUD = 115200UL;
const unsigned long STREAM_INTERVAL_MS = 500UL;
const int ANALOG_SAMPLES = 6;

const unsigned long ULTRA_TIMEOUT_US = 25000UL;
const float ULTRA_MIN_CM = 2.0f;
const float ULTRA_MAX_CM = 400.0f;
const float OBSTACLE_LED_CM = 20.0f;

const uint8_t BUZZER_PIN = 41;   // PG0
const uint8_t LED_DATA_PIN = 24; // WS2812B data pin
const uint8_t NUM_LEDS = 24;     // change if your panel has different count
const uint8_t LED_BRIGHTNESS = 96;

const uint8_t SERVO_PIN = 39;
const int SERVO_CENTER_DEG = 90;
const int SERVO_CLOCKWISE_DEG = 140;
const int SERVO_STEP_DEG = 2;
const int SERVO_STEP_DELAY_MS = 55;

// ============================================================
// Motor config (merged from wall_bounce.ino)
// ============================================================
const uint8_t M1_PWM = 12;  const uint8_t M1_IN1 = 34;  const uint8_t M1_IN2 = 35;
const uint8_t M2_PWM = 8;   const uint8_t M2_IN1 = 37;  const uint8_t M2_IN2 = 36;
const uint8_t M3_PWM = 9;   const uint8_t M3_IN1 = 43;  const uint8_t M3_IN2 = 42;
const uint8_t M4_PWM = 5;   const uint8_t M4_IN1 = A4;  const uint8_t M4_IN2 = A5;

const int MOTOR_SIGN[4]  = { -1, +1, -1, +1 };
const int MOTOR_TRIM[4]  = { 0, 0, 0, 0 };
int motorRuntimeTrim[4]  = { 0, 0, 0, 0 };
const int SIDE_BALANCE   = 0;

const int   PWM_FORWARD        = 42;
const int   PWM_BACKUP         = 34;
const int   PWM_TURN_90        = 28;
const float FORWARD_RATIO[4]   = { 0.9650f, 0.8462f, 0.9650f, 0.8462f };
const float BACKWARD_RATIO[4]  = { 1.0000f, 0.8462f, 1.0000f, 0.8462f };
const unsigned long TURN_LEFT_90_MS  = 2400UL;
const unsigned long TURN_RIGHT_90_MS = 2550UL;
const unsigned long STOP_SETTLE_MS   = 150UL;

// Front ultrasonic for patrol (U4)
const uint8_t PATROL_FRONT_TRIG = 30;  // PC7 / D30
const uint8_t PATROL_FRONT_ECHO = 29;  // PA7 / D29
const float   FRONT_WALL_STOP_CM = 25.0f;

// ============================================================
// Bluetooth remote link (HC-05 on Serial3)
// Mega Serial3: TX=14, RX=15
// HC-05 default baud: 9600
// ============================================================
const unsigned long BT_BAUD = 9600UL;

// ============================================================
// Mode timing
// ============================================================
const unsigned long FIRE_ALERT_DURATION_MS     = 10000UL;
const unsigned long SECURITY_ALERT_DURATION_MS = 10000UL;
const unsigned long VERIF_TIMEOUT_MS           = 8000UL;  // fresh window after scan
const unsigned long VERIF_SERVO_TRIGGER_MS     = 6000UL;  // +3 s gap so detected_person.wav finishes before approach_camera.wav starts
const unsigned long VERIF_AUDIO_WAIT_MS        = 3000UL;  // approach_camera.wav duration
const unsigned long VERIF_SERVO_HOLD_MS        = 3000UL;  // hold at 130° with white LED
const int           VERIF_SERVO_MAX_DEG        = 130;

// ============================================================
// OLED config (friend's code logic reflected)
// SSD1306 I2C, try 0x3C first, then 0x3D
// Mega: SDA=20, SCL=21
// ============================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR_PRIMARY 0x3C
#define OLED_ADDR_FALLBACK 0x3D

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledReady = false;
uint8_t activeOledAddress = 0;

// ============================================================
// Hazard thresholds from measured values
// ============================================================
const int SIDE_FLAME_THRESHOLD = 300;   // IR_1 / IR_2 / IR_BACK: lower = flame
const int FRONT_ACTIVE_TH      = 300;    // IR_5_x: higher = active
const int FRONT_STRONG_TH      = 400;   // IR_5_x: higher = strong
const int GAS_THRESHOLD        = 150;    // MQ-2 analog threshold
const int GAS_STABLE_COUNT     = 3;
// Front IR sensors see ambient IR (body heat, lighting) and need debounce.
// A real flame stays above threshold for many consecutive reads;
// a noise spike is usually 1-2 reads.  At HAZARD_SAMPLE_INTERVAL_MS=100 ms,
// FRONT_STABLE_COUNT=8 means ~800 ms sustained reading before alert fires.
const int FRONT_STABLE_COUNT   = 8;

// ============================================================
// Fast urgent blink timing
// lower = faster blink
// ============================================================
const unsigned long HAZARD_SAMPLE_INTERVAL_MS = 100UL;
const unsigned long ALERT_BLINK_MS = 50UL;

// ============================================================
// Sensor structs
// ============================================================
struct DualSensor
{
  const char* name;
  const char* digitalLabel;
  const char* analogLabel;
  uint8_t digitalPin;
  uint8_t analogPin;
};

struct UltrasonicSensor
{
  const char* name;
  const char* trigLabel;
  const char* echoLabel;
  uint8_t trigPin;
  uint8_t echoPin;
};

const DualSensor kDualSensors[] = {
  {"IR_1",    "PL1/D48", "A2",  48, A2},
  {"IR_2",    "PL2/D47", "A3",  47, A3},
  {"IR_5_1",  "PB5/D11", "A8",  11, A8},
  {"IR_5_2",  "PB4/D10", "A9",  10, A9},
  {"IR_5_3",  "PH4/D7",  "A10", 7,  A10},
  {"IR_5_4",  "PH3/D6",  "A11", 6,  A11},
  {"IR_5_5",  "PG5/D4",  "A12", 4,  A12},
  {"IR_BACK", "PL4/D45", "A6",  45, A6},
  {"GAS",     "PL5/D44", "A0",  44, A0},
};

const UltrasonicSensor kUltrasonicSensors[] = {
  {"U1", "PC4/D33", "PC5/D32", 33, 32},
  {"U2", "PA6/D28", "PA3/D25", 28, 25},
  {"U3", "PL3/D46", "PB7/D13", 46, 13},
  {"U4", "PC7/D30", "PA7/D29", 30, 29},
};

enum DualSensorIndex
{
  IDX_IR1 = 0,
  IDX_IR2,
  IDX_IR5_1,
  IDX_IR5_2,
  IDX_IR5_3,
  IDX_IR5_4,
  IDX_IR5_5,
  IDX_IR_BACK,
  IDX_GAS
};

// ============================================================
// Robot mode + motor struct
// ============================================================
enum RobotMode { MODE_PATROL, MODE_FIRE_ALERT, MODE_VERIFICATION, MODE_SECURITY_ALERT };
enum PatrolSubState { PSUB_FORWARD, PSUB_TURNING };

struct MotorPins { uint8_t pwm; uint8_t in1; uint8_t in2; };
const MotorPins MOTORS[4] = {
  { M1_PWM, M1_IN1, M1_IN2 },
  { M2_PWM, M2_IN1, M2_IN2 },
  { M3_PWM, M3_IN1, M3_IN2 },
  { M4_PWM, M4_IN1, M4_IN2 }
};

// ============================================================
// Globals
// ============================================================
bool streamEnabled = false;  // sensor stream off by default — press 'p' to toggle
unsigned long lastStreamMs = 0;
unsigned long lastHazardSampleMs = 0;
unsigned long lastBlinkMs = 0;
bool alertBlinkOn = false;

CRGB statusLeds[NUM_LEDS];
Servo testServo;

int gAnalogValues[sizeof(kDualSensors) / sizeof(kDualSensors[0])] = {0};
int gDigitalValues[sizeof(kDualSensors) / sizeof(kDualSensors[0])] = {0};

int gGasCount = 0;
bool gGasAlert = false;
int gFrontCount = 0;   // consecutive front-IR readings above FRONT_ACTIVE_TH
bool gFrontAlert = false;
bool gHazardDetected = false;
const char* gDirection = "NONE";
const char* gHazardType = "NONE";

// Mode state
RobotMode      gMode            = MODE_PATROL;
unsigned long  gModeStartMs     = 0;
bool           gVerifServoScanned = false;
PatrolSubState gPatrolSub       = PSUB_FORWARD;
unsigned long  gPatrolTurnStartMs = 0;

// ============================================================
// Function declarations
// ============================================================
int readAnalogAverage(uint8_t analogPin, int samples = ANALOG_SAMPLES);
float readDistanceCm(uint8_t trigPin, uint8_t echoPin);
float readDistanceMedian(uint8_t trigPin, uint8_t echoPin);

// LED / servo
void setStatusLed(const CRGB& color);
void runLedTest();
void centerServo();
void runServoSweep();

// Serial
void printHelp();
void printSnapshot();
void printHazardTrigger();
void dispatchSingleChar(char cmd);
void dispatchJetsonLine(const char* line);
void handleSerial();

// Hazard
bool sideFlameTriggered(int value);
bool frontActive(int value);
bool frontStrong(int value);
void readAllHazardSensors();
const char* getFrontDirectionFromCurrentReadings();
const char* getOverallDirectionFromCurrentReadings();
const char* getHazardTypeFromState(bool hazard, bool gasAlert, const char* direction);
void updateHazardState(bool forceRead = false);

// OLED
bool isI2cPresent(uint8_t address);
bool beginOledAtAddress(uint8_t address);
void initOled();
void drawNormalOled();
void drawAlertOled(bool visible);
void drawVerificationOled();
void drawSecurityAlertOled(bool visible);

// Motors
void setupMotorPins();
void writeMotorRaw(uint8_t pwmPin, uint8_t in1, uint8_t in2, int signedPwm);
void writeMotorByIndex(uint8_t index, int signedPwm);
void drive4(int m1, int m2, int m3, int m4);
void driveTank(int leftPwm, int rightPwm);
int  scaleMotionPwm(int basePwm, float ratio);
void driveForward(int pwm);
void driveBackward(int pwm);
void rotateLeft(int pwm);
void rotateRight(int pwm);
void stopMotors();
void rotateNinetyDegreesRight();
void rotateNinetyDegreesLeft();

// Bluetooth remote link
void btSend(const char* msg);
void btSendFireAlert();
void btSendSecurityAlert();

// Mode state machine
void enterPatrol();
void enterFireAlert();
void enterVerification();
void enterSecurityAlert();
void runPatrol();
void runFireAlert();
void runVerification();
void runSecurityAlert();
void runCurrentMode();

// ============================================================
// Utility functions
// ============================================================
int readAnalogAverage(uint8_t analogPin, int samples)
{
  long total = 0;
  for (int i = 0; i < samples; ++i)
  {
    total += analogRead(analogPin);
    delay(2);
  }
  return (int)(total / samples);
}

float readDistanceCm(uint8_t trigPin, uint8_t echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long duration = pulseIn(echoPin, HIGH, ULTRA_TIMEOUT_US);
  if (duration == 0UL)
  {
    return -1.0f;
  }

  const float distanceCm = duration * 0.0343f * 0.5f;
  if (distanceCm < ULTRA_MIN_CM || distanceCm > ULTRA_MAX_CM)
  {
    return -1.0f;
  }

  return distanceCm;
}

float readDistanceMedian(uint8_t trigPin, uint8_t echoPin)
{
  float a = readDistanceCm(trigPin, echoPin);
  delay(8);
  float b = readDistanceCm(trigPin, echoPin);
  delay(8);
  float c = readDistanceCm(trigPin, echoPin);

  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }

  return b;
}

void setStatusLed(const CRGB& color)
{
  fill_solid(statusLeds, NUM_LEDS, color);
  FastLED.show();
}

void runLedTest()
{
  Serial.println(F("[LED] FastLED color test on D24"));

  setStatusLed(CRGB::Red);
  delay(180);
  setStatusLed(CRGB::Green);
  delay(180);
  setStatusLed(CRGB::Blue);
  delay(180);
  setStatusLed(CRGB::White);
  delay(180);
  setStatusLed(CRGB::Black);
  delay(120);
}

void centerServo()
{
  testServo.write(SERVO_CENTER_DEG);
  delay(350);
  Serial.println(F("[SERVO] centered at 90 deg on D39"));
}

void runServoSweep()
{
  Serial.println(F("[SERVO] SG90 clockwise-only test on D39"));

  for (int angle = SERVO_CENTER_DEG; angle <= SERVO_CLOCKWISE_DEG; angle += SERVO_STEP_DEG)
  {
    testServo.write(angle);
    delay(SERVO_STEP_DELAY_MS);
  }

  delay(200);
  testServo.write(SERVO_CENTER_DEG);
  delay(350);

  Serial.println(F("[SERVO] clockwise test done, centered"));
}

// ============================================================
// Motor control (merged from wall_bounce.ino)
// ============================================================
void setupMotorPins()
{
  const uint8_t pins[][3] = {
    { M1_PWM, M1_IN1, M1_IN2 },
    { M2_PWM, M2_IN1, M2_IN2 },
    { M3_PWM, M3_IN1, M3_IN2 },
    { M4_PWM, M4_IN1, M4_IN2 }
  };
  for (uint8_t i = 0; i < 4; ++i)
  {
    pinMode(pins[i][0], OUTPUT);
    pinMode(pins[i][1], OUTPUT);
    pinMode(pins[i][2], OUTPUT);
  }
}

void writeMotorRaw(uint8_t pwmPin, uint8_t in1, uint8_t in2, int signedPwm)
{
  signedPwm = constrain(signedPwm, -255, 255);
  if (signedPwm > 0)       { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  analogWrite(pwmPin,  signedPwm); }
  else if (signedPwm < 0)  { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); analogWrite(pwmPin, -signedPwm); }
  else                     { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  analogWrite(pwmPin, 0); }
}

void writeMotorByIndex(uint8_t index, int signedPwm)
{
  if (signedPwm != 0)
  {
    int dir = (signedPwm > 0) ? 1 : -1;
    int mag = constrain(abs(signedPwm) + MOTOR_TRIM[index] + motorRuntimeTrim[index], 0, 255);
    signedPwm = dir * mag;
  }
  signedPwm *= MOTOR_SIGN[index];
  writeMotorRaw(MOTORS[index].pwm, MOTORS[index].in1, MOTORS[index].in2, signedPwm);
}

int scaleMotionPwm(int basePwm, float ratio)
{
  return constrain((int)((float)basePwm * ratio + 0.5f), 0, 255);
}

void drive4(int m1, int m2, int m3, int m4)
{
  writeMotorByIndex(0, m1);
  writeMotorByIndex(1, m2);
  writeMotorByIndex(2, m3);
  writeMotorByIndex(3, m4);
}

void driveTank(int leftPwm, int rightPwm)
{
  drive4(leftPwm - SIDE_BALANCE, rightPwm + SIDE_BALANCE,
         leftPwm - SIDE_BALANCE, rightPwm + SIDE_BALANCE);
}

void stopMotors()   { drive4(0, 0, 0, 0); }

void driveForward(int pwm)
{
  drive4(scaleMotionPwm(pwm, FORWARD_RATIO[0]),  scaleMotionPwm(pwm, FORWARD_RATIO[1]),
         scaleMotionPwm(pwm, FORWARD_RATIO[2]),  scaleMotionPwm(pwm, FORWARD_RATIO[3]));
}

void driveBackward(int pwm)
{
  drive4(-scaleMotionPwm(pwm, BACKWARD_RATIO[0]), -scaleMotionPwm(pwm, BACKWARD_RATIO[1]),
         -scaleMotionPwm(pwm, BACKWARD_RATIO[2]), -scaleMotionPwm(pwm, BACKWARD_RATIO[3]));
}

void rotateLeft(int pwm)  { driveTank( pwm, -pwm); }
void rotateRight(int pwm) { driveTank(-pwm,  pwm); }

void rotateNinetyDegreesRight()
{
  rotateRight(PWM_TURN_90);
  delay(TURN_RIGHT_90_MS);
  stopMotors();
  delay(STOP_SETTLE_MS);
}

void rotateNinetyDegreesLeft()
{
  rotateLeft(PWM_TURN_90);
  delay(TURN_LEFT_90_MS);
  stopMotors();
  delay(STOP_SETTLE_MS);
}

// ============================================================
// Hazard logic
// ============================================================
bool sideFlameTriggered(int value)
{
  return value < SIDE_FLAME_THRESHOLD;
}

bool frontActive(int value)
{
  return value > FRONT_ACTIVE_TH;
}

bool frontStrong(int value)
{
  return value > FRONT_STRONG_TH;
}

void readAllHazardSensors()
{
  for (size_t i = 0; i < (sizeof(kDualSensors) / sizeof(kDualSensors[0])); ++i)
  {
    gAnalogValues[i] = readAnalogAverage(kDualSensors[i].analogPin);
    gDigitalValues[i] = digitalRead(kDualSensors[i].digitalPin);
  }
}

const char* getFrontDirectionFromCurrentReadings()
{
  const int f1 = gAnalogValues[IDX_IR5_1];
  const int f2 = gAnalogValues[IDX_IR5_2];
  const int f3 = gAnalogValues[IDX_IR5_3];
  const int f4 = gAnalogValues[IDX_IR5_4];
  const int f5 = gAnalogValues[IDX_IR5_5];

  const bool a1 = frontActive(f1);
  const bool a2 = frontActive(f2);
  const bool a3 = frontActive(f3);
  const bool a4 = frontActive(f4);
  const bool a5 = frontActive(f5);

  const bool s1 = frontStrong(f1);
  const bool s2 = frontStrong(f2);
  const bool s3 = frontStrong(f3);
  const bool s4 = frontStrong(f4);
  const bool s5 = frontStrong(f5);

  if (s3 && !s1 && !s5) return "FRONT_CENTER";

  if (s1 || s2)
  {
    if (s3) return "FRONT_LEFT_CENTER";
    return "FRONT_LEFT";
  }

  if (s4 || s5)
  {
    if (s3) return "FRONT_RIGHT_CENTER";
    return "FRONT_RIGHT";
  }

  if (a3) return "FRONT_CENTER";
  if ((a1 || a2) && (a4 || a5)) return "FRONT_WIDE";

  return "NONE";
}

const char* getOverallDirectionFromCurrentReadings()
{
  const bool leftTrig  = sideFlameTriggered(gAnalogValues[IDX_IR1]);
  const bool rightTrig = sideFlameTriggered(gAnalogValues[IDX_IR2]);
  const bool backTrig  = sideFlameTriggered(gAnalogValues[IDX_IR_BACK]);

  if (leftTrig && !rightTrig && !backTrig) return "LEFT";
  if (rightTrig && !leftTrig && !backTrig) return "RIGHT";
  if (backTrig && !leftTrig && !rightTrig) return "REAR";

  if (leftTrig && rightTrig && !backTrig) return "BOTH_SIDE";
  if (leftTrig && backTrig && !rightTrig) return "LEFT_REAR";
  if (rightTrig && backTrig && !leftTrig) return "RIGHT_REAR";
  if (leftTrig && rightTrig && backTrig) return "MULTI_SIDE_REAR";

  return getFrontDirectionFromCurrentReadings();
}

const char* getHazardTypeFromState(bool hazard, bool gasAlert, const char* direction)
{
  if (!hazard && !gasAlert && strcmp(direction, "NONE") == 0) return "NONE";
  if (strcmp(direction, "NONE") != 0 && !gasAlert) return "FLAME";
  if (strcmp(direction, "NONE") == 0 && gasAlert) return "SMOKE_GAS";
  return "BOTH";
}

void updateHazardState(bool forceRead)
{
  const unsigned long nowMs = millis();

  if (!forceRead && (nowMs - lastHazardSampleMs) < HAZARD_SAMPLE_INTERVAL_MS)
  {
    return;
  }

  lastHazardSampleMs = nowMs;
  readAllHazardSensors();

  const int gasValue = gAnalogValues[IDX_GAS];
  if (gasValue >= GAS_THRESHOLD)
  {
    gGasCount++;
  }
  else
  {
    gGasCount = 0;
  }

  gGasAlert = (gGasCount >= GAS_STABLE_COUNT);

  // Front IR debounce — require FRONT_STABLE_COUNT consecutive reads above
  // FRONT_ACTIVE_TH before treating any front sensor as a real flame detection.
  // This prevents single-sample ambient spikes (body heat, lighting) from
  // falsely triggering FIRE_ALERT.
  const bool anyFrontActive = frontActive(gAnalogValues[IDX_IR5_1]) ||
                              frontActive(gAnalogValues[IDX_IR5_2]) ||
                              frontActive(gAnalogValues[IDX_IR5_3]) ||
                              frontActive(gAnalogValues[IDX_IR5_4]) ||
                              frontActive(gAnalogValues[IDX_IR5_5]);
  if (anyFrontActive) { gFrontCount++; } else { gFrontCount = 0; }
  gFrontAlert = (gFrontCount >= FRONT_STABLE_COUNT);

  gDirection = getOverallDirectionFromCurrentReadings();

  // If direction was set purely by front sensors but debounce hasn't confirmed,
  // treat it as no hazard yet (side/back sensors are not gated here).
  if (!gFrontAlert && strncmp(gDirection, "FRONT", 5) == 0)
  {
    gDirection = "NONE";
  }

  gHazardDetected = (strcmp(gDirection, "NONE") != 0) || gGasAlert;
  gHazardType = getHazardTypeFromState(gHazardDetected, gGasAlert, gDirection);
}

// ============================================================
// OLED init / draw
// ============================================================
bool isI2cPresent(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool beginOledAtAddress(uint8_t address)
{
  if (!isI2cPresent(address))
  {
    Serial.print(F("[OLED] No I2C ACK at 0x"));
    Serial.println(address, HEX);
    return false;
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, address))
  {
    return false;
  }

  activeOledAddress = address;
  Serial.print(F("[OLED] SSD1306 found at 0x"));
  Serial.println(address, HEX);
  return true;
}

void initOled()
{
  Wire.begin();
  Wire.setClock(100000UL);

  oledReady = beginOledAtAddress(OLED_ADDR_PRIMARY);

  if (!oledReady)
  {
    oledReady = beginOledAtAddress(OLED_ADDR_FALLBACK);
  }

  if (!oledReady)
  {
    Serial.println(F("[OLED] SSD1306 init failed. Check 0x3C/0x3D, SDA=20, SCL=21."));
    return;
  }

  display.clearDisplay();
  display.display();
}

void drawNormalOled()
{
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(10, 8);
  display.println(F("PATROL"));

  display.setCursor(25, 36);
  display.println(F("SAFE"));

  display.display();
}

void drawAlertOled(bool visible)
{
  if (!oledReady) return;

  display.clearDisplay();

  if (!visible)
  {
    display.display();
    return;
  }

  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(28, 0);
  display.println(F("!!! ALERT !!!"));

  display.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE);

  display.setTextSize(2);

  if (strcmp(gHazardType, "BOTH") == 0)
  {
    display.setCursor(10, 18);
    display.println(F("FIRE+GAS"));
    display.setTextSize(1);
    display.setCursor(22, 44);
    display.println(F("DETECTED"));
  }
  else if (strcmp(gHazardType, "SMOKE_GAS") == 0)
  {
    display.setCursor(22, 18);
    display.println(F("GAS"));
    display.setCursor(4, 42);
    display.println(F("DETECTED"));
  }
  else
  {
    display.setCursor(18, 18);
    display.println(F("FIRE"));
    display.setCursor(4, 42);
    display.println(F("DETECTED"));
  }

  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(F("DIR: "));
  display.println(gDirection);

  display.display();
}

void drawVerificationOled()
{
  if (!oledReady) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(20, 0);
  display.println(F("[ PERSON DETECTED ]"));
  display.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(10, 18);
  display.println(F("SCANNING"));
  display.setTextSize(1);
  display.setCursor(10, 52);
  display.println(F("Please face camera"));
  display.display();
}

void drawSecurityAlertOled(bool visible)
{
  if (!oledReady) return;
  display.clearDisplay();
  if (!visible) { display.display(); return; }
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(28, 0);
  display.println(F("!!! ALERT !!!"));
  display.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(4, 18);
  display.println(F("INTRUDER"));
  display.setCursor(4, 42);
  display.println(F("DETECTED"));
  display.display();
}

// ============================================================
// LED + OLED fast alert update (legacy — kept for reference)
// Replaced by runCurrentMode() in the main loop.
// ============================================================
void updateAlertOutputs()
{
  updateHazardState(false);

  if (gHazardDetected)
  {
    const unsigned long nowMs = millis();
    if (nowMs - lastBlinkMs >= ALERT_BLINK_MS)
    {
      lastBlinkMs = nowMs;
      alertBlinkOn = !alertBlinkOn;

      if (alertBlinkOn)
      {
        setStatusLed(CRGB::Red);
      }
      else
      {
        setStatusLed(CRGB::Black);
      }

      drawAlertOled(alertBlinkOn);
    }
  }
  else
  {
    alertBlinkOn = false;
    setStatusLed(CRGB::White);
    drawNormalOled();
  }
}

// ============================================================
// Bluetooth remote link
// ============================================================

void btSend(const char* msg)
{
  Serial3.println(msg);
  Serial.print(F("[BT->] "));
  Serial.println(msg);
}

void btSendFireAlert()
{
  char buf[100];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"fire\",\"subtype\":\"%s\",\"dir\":\"%s\",\"ts\":%lu}",
    gHazardType, gDirection, millis());
  btSend(buf);
}

void btSendSecurityAlert()
{
  char buf[48];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"intruder\",\"ts\":%lu}", millis());
  btSend(buf);
}

// ============================================================
// Mode state machine
// ============================================================

void enterPatrol()
{
  gMode = MODE_PATROL;
  gModeStartMs = millis();
  gPatrolSub = PSUB_FORWARD;
  alertBlinkOn = false;
  noTone(BUZZER_PIN);
  setStatusLed(CRGB::White);
  drawNormalOled();
  Serial.println(F("[MODE] PATROL"));
  Serial.println(F("MODE:PATROL"));
}

void enterFireAlert()
{
  gMode = MODE_FIRE_ALERT;
  gModeStartMs = millis();
  stopMotors();
  // Detach servo so FastLED interrupt blocking (every 50 ms for LED blink)
  // does not corrupt the servo PWM signal and cause twitching.
  testServo.detach();
  alertBlinkOn = false;
  lastBlinkMs = 0;
  Serial.println(F("[MODE] FIRE_ALERT"));
  Serial.println(F("MODE:FIRE_ALERT"));
  printHazardTrigger();   // log exactly which sensors fired and their raw values
  Serial.println(F("PLAY:alert_fire"));
  btSendFireAlert();
}

void enterVerification()
{
  gMode = MODE_VERIFICATION;
  gModeStartMs = millis();
  gVerifServoScanned = false;
  stopMotors();
  // Re-attach servo before using it (may have been detached during FIRE_ALERT).
  testServo.attach(SERVO_PIN);
  centerServo();
  noTone(BUZZER_PIN);
  setStatusLed(CRGB(255, 80, 0));  // orange
  drawVerificationOled();
  Serial.println(F("[MODE] VERIFICATION"));
  Serial.println(F("MODE:VERIFICATION"));
  Serial.println(F("PLAY:detected_person"));
}

void enterSecurityAlert()
{
  gMode = MODE_SECURITY_ALERT;
  gModeStartMs = millis();
  stopMotors();
  // Re-attach in case servo was detached during FIRE_ALERT, center, then
  // detach again so FastLED blinking does not cause twitching here either.
  testServo.attach(SERVO_PIN);
  centerServo();
  testServo.detach();
  alertBlinkOn = false;
  lastBlinkMs = 0;
  Serial.println(F("[MODE] SECURITY_ALERT"));
  Serial.println(F("MODE:SECURITY_ALERT"));
  Serial.println(F("PLAY:alert_intruder"));
  btSendSecurityAlert();
}

// ── mode runners (called every loop tick) ─────────────────────────────────

void runPatrol()
{
  // fire / gas takes priority — enter FIRE_ALERT immediately
  if (gHazardDetected)
  {
    enterFireAlert();
    return;
  }

  const unsigned long nowMs = millis();

  if (gPatrolSub == PSUB_FORWARD)
  {
    const float frontCm = readDistanceMedian(PATROL_FRONT_TRIG, PATROL_FRONT_ECHO);

    if (frontCm > 0.0f && frontCm <= FRONT_WALL_STOP_CM)
    {
      stopMotors();
      delay(STOP_SETTLE_MS);
      gPatrolSub = PSUB_TURNING;
      gPatrolTurnStartMs = nowMs;
      rotateRight(PWM_TURN_90);
    }
    else
    {
      driveForward(PWM_FORWARD);
    }
  }
  else  // PSUB_TURNING
  {
    if (nowMs - gPatrolTurnStartMs >= TURN_RIGHT_90_MS)
    {
      stopMotors();
      delay(STOP_SETTLE_MS);
      gPatrolSub = PSUB_FORWARD;
    }
  }
}

void runFireAlert()
{
  const unsigned long nowMs = millis();

  if (nowMs - lastBlinkMs >= ALERT_BLINK_MS)
  {
    lastBlinkMs = nowMs;
    alertBlinkOn = !alertBlinkOn;

    if (alertBlinkOn)
    {
      setStatusLed(CRGB::Red);
      tone(BUZZER_PIN, 2200, (int)ALERT_BLINK_MS - 5);
    }
    else
    {
      setStatusLed(CRGB::Black);
    }
    drawAlertOled(alertBlinkOn);
  }

  if (nowMs - gModeStartMs >= FIRE_ALERT_DURATION_MS)
  {
    noTone(BUZZER_PIN);
    enterPatrol();
  }
}

void runVerification()
{
  const unsigned long nowMs = millis();
  const unsigned long elapsed = nowMs - gModeStartMs;

  // ── Phase 1: trigger servo scan after VERIF_SERVO_TRIGGER_MS ──────────────
  if (!gVerifServoScanned && elapsed >= VERIF_SERVO_TRIGGER_MS)
  {
    gVerifServoScanned = true;

    // 1. Red LED + play audio, then wait for clip to finish
    setStatusLed(CRGB::Red);
    Serial.println(F("PLAY:approach_camera"));
    delay(VERIF_AUDIO_WAIT_MS);

    // 2. Sweep servo slowly from 90° to 130°
    for (int a = SERVO_CENTER_DEG; a <= VERIF_SERVO_MAX_DEG; a += SERVO_STEP_DEG)
    {
      testServo.write(a);
      delay(SERVO_STEP_DELAY_MS);
    }

    // 3. At 130°: white LED, hold 3 s
    setStatusLed(CRGB::White);
    delay(VERIF_SERVO_HOLD_MS);

    // 4. Return servo to 90°
    centerServo();

    // 5. Flush serial buffer — FACE_VERIFIED may have arrived during the
    //    blocking delays above and is waiting to be read.
    handleSerial();

    // 6. If Jetson already resolved the mode (FACE_VERIFIED → enterPatrol
    //    was called inside handleSerial), we are done.
    if (gMode != MODE_VERIFICATION) return;

    // 7. Scan done but no answer yet — reset timer to give Jetson
    //    VERIF_TIMEOUT_MS to send FACE_VERIFIED / FACE_UNKNOWN / FACE_TIMEOUT.
    Serial.println(F("[VERIF] scan done — waiting for Jetson response"));
    gModeStartMs = millis();
    return;
  }

  // ── Phase 2: post-scan response timeout ───────────────────────────────────
  // Only runs after the servo scan has completed (gVerifServoScanned == true)
  // and Jetson has not responded within VERIF_TIMEOUT_MS.
  if (gVerifServoScanned && elapsed >= VERIF_TIMEOUT_MS)
  {
    Serial.println(F("[VERIF] timeout — no Jetson response after scan"));
    enterSecurityAlert();
  }
}

void runSecurityAlert()
{
  const unsigned long nowMs = millis();

  if (nowMs - lastBlinkMs >= ALERT_BLINK_MS)
  {
    lastBlinkMs = nowMs;
    alertBlinkOn = !alertBlinkOn;

    if (alertBlinkOn)
    {
      setStatusLed(CRGB::Red);
      tone(BUZZER_PIN, 3500, (int)ALERT_BLINK_MS - 5);  // higher tone than fire
    }
    else
    {
      setStatusLed(CRGB::Black);
    }
    drawSecurityAlertOled(alertBlinkOn);
  }

  if (nowMs - gModeStartMs >= SECURITY_ALERT_DURATION_MS)
  {
    noTone(BUZZER_PIN);
    enterPatrol();
  }
}

void runCurrentMode()
{
  switch (gMode)
  {
    case MODE_PATROL:         runPatrol();        break;
    case MODE_FIRE_ALERT:     runFireAlert();     break;
    case MODE_VERIFICATION:   runVerification();  break;
    case MODE_SECURITY_ALERT: runSecurityAlert(); break;
  }
}

// ============================================================
// Serial output
// ============================================================
void printHelp()
{
  Serial.println(F(""));
  Serial.println(F("=== ELEC3848 Hazard Monitor + FAST LED/OLED Alert ==="));
  Serial.println(F("Serial Monitor baud: 115200"));
  Serial.println(F("Commands:"));
  Serial.println(F("  h : print help"));
  Serial.println(F("  o : print one snapshot now"));
  Serial.println(F("  p : pause/resume auto streaming"));
  Serial.println(F("  b : short buzzer beep"));
  Serial.println(F("  l : FastLED color test on D24"));
  Serial.println(F("  v : SG90 servo sweep on D39"));
  Serial.println(F("  c : center SG90 servo at 90 deg"));
  Serial.println(F(""));
  Serial.println(F("Normal  -> LED WHITE, OLED PATROL SAFE"));
  Serial.println(F("Hazard  -> LED RED fast blink, OLED fast alert blink"));
  Serial.println(F(""));
}

void printSnapshot()
{
  updateHazardState(true);

  bool obstacleWarning = false;

  Serial.println(F("------------------------------------------------------------"));
  Serial.println(F("[Analog + Digital sensors]"));

  for (size_t i = 0; i < (sizeof(kDualSensors) / sizeof(kDualSensors[0])); ++i)
  {
    const DualSensor& sensor = kDualSensors[i];

    Serial.print(sensor.name);
    Serial.print(F(": A="));
    Serial.print(gAnalogValues[i]);
    Serial.print(F(" D="));
    Serial.print(gDigitalValues[i]);
    Serial.print(F("  ("));
    Serial.print(sensor.digitalLabel);
    Serial.print(F(", "));
    Serial.print(sensor.analogLabel);
    Serial.println(F(")"));
  }

  Serial.println(F(""));
  Serial.println(F("[Ultrasonic sensors]"));

  for (size_t i = 0; i < (sizeof(kUltrasonicSensors) / sizeof(kUltrasonicSensors[0])); ++i)
  {
    const UltrasonicSensor& sensor = kUltrasonicSensors[i];
    const float distanceCm = readDistanceMedian(sensor.trigPin, sensor.echoPin);

    Serial.print(sensor.name);
    Serial.print(F(": "));
    if (distanceCm < 0.0f)
    {
      Serial.print(F("NO_ECHO"));
    }
    else
    {
      Serial.print(distanceCm, 1);
      Serial.print(F(" cm"));
      if (distanceCm <= OBSTACLE_LED_CM)
      {
        obstacleWarning = true;
      }
    }
    Serial.print(F("  (trig "));
    Serial.print(sensor.trigLabel);
    Serial.print(F(", echo "));
    Serial.print(sensor.echoLabel);
    Serial.println(F(")"));

    delay(20);
  }

  Serial.println(F(""));
  Serial.println(F("[Hazard summary]"));
  Serial.print(F("Direction="));
  Serial.print(gDirection);
  Serial.print(F(" | HazardType="));
  Serial.print(gHazardType);
  Serial.print(F(" | GasAlert="));
  Serial.print(gGasAlert ? F("YES") : F("NO"));
  Serial.print(F(" | HazardDetected="));
  Serial.println(gHazardDetected ? F("YES") : F("NO"));

  Serial.print(F("[LED] status="));
  Serial.println(gHazardDetected ? F("RED_FAST_BLINK") : F("WHITE"));

  Serial.print(F("[OLED] status="));
  Serial.println(gHazardDetected ? F("FAST_ALERT_BLINK") : F("PATROL_SAFE"));

  if (obstacleWarning)
  {
    Serial.println(F("[ULTRASONIC] obstacle within warning distance"));
  }
}

// ============================================================
// Hazard trigger report — printed once when FIRE_ALERT is entered.
// Shows every sensor that exceeded its threshold so you can tune values.
// ============================================================
void printHazardTrigger()
{
  Serial.println(F(""));
  Serial.println(F("[HAZARD TRIGGER] Sensors that exceeded threshold:"));

  // Side / back flame (IR_1, IR_2, IR_BACK) — lower value = more flame
  const struct { uint8_t idx; const char* label; } sideSensors[] = {
    { IDX_IR1,    "IR_1  (LEFT) " },
    { IDX_IR2,    "IR_2  (RIGHT)" },
    { IDX_IR_BACK,"IR_BACK      " },
  };
  for (uint8_t i = 0; i < 3; ++i)
  {
    int val = gAnalogValues[sideSensors[i].idx];
    if (sideFlameTriggered(val))
    {
      Serial.print(F("  "));
      Serial.print(sideSensors[i].label);
      Serial.print(F(" A="));
      Serial.print(val);
      Serial.print(F("  threshold < "));
      Serial.println(SIDE_FLAME_THRESHOLD);
    }
  }

  // Front array (IR_5_1 … IR_5_5) — higher value = more flame
  const struct { uint8_t idx; const char* label; } frontSensors[] = {
    { IDX_IR5_1, "IR_5_1 (F-L2)" },
    { IDX_IR5_2, "IR_5_2 (F-L1)" },
    { IDX_IR5_3, "IR_5_3 (F-CTR)" },
    { IDX_IR5_4, "IR_5_4 (F-R1)" },
    { IDX_IR5_5, "IR_5_5 (F-R2)" },
  };
  for (uint8_t i = 0; i < 5; ++i)
  {
    int val = gAnalogValues[frontSensors[i].idx];
    if (frontActive(val))
    {
      Serial.print(F("  "));
      Serial.print(frontSensors[i].label);
      Serial.print(F(" A="));
      Serial.print(val);
      if (frontStrong(val))
      {
        Serial.print(F("  STRONG (> "));
        Serial.print(FRONT_STRONG_TH);
        Serial.print(F(")"));
      }
      else
      {
        Serial.print(F("  active (> "));
        Serial.print(FRONT_ACTIVE_TH);
        Serial.print(F(")"));
      }
      Serial.println();
    }
  }

  // Gas sensor
  int gasVal = gAnalogValues[IDX_GAS];
  if (gGasAlert)
  {
    Serial.print(F("  GAS           A="));
    Serial.print(gasVal);
    Serial.print(F("  threshold >= "));
    Serial.println(GAS_THRESHOLD);
  }

  // Summary
  Serial.print(F("[HAZARD TRIGGER] type="));
  Serial.print(gHazardType);
  Serial.print(F("  dir="));
  Serial.print(gDirection);
  Serial.print(F("  frontCount="));
  Serial.print(gFrontCount);
  Serial.print(F("/"));
  Serial.println(FRONT_STABLE_COUNT);
  Serial.println(F(""));
}

// ============================================================
// Serial line reader — handles both single-char interactive
// commands and multi-token Jetson event strings.
// ============================================================

static char  sLineBuf[80];
static uint8_t sLineLen = 0;

void dispatchSingleChar(char cmd)
{
  switch (cmd)
  {
    case 'h':
      printHelp();
      break;

    case 'o':
      printSnapshot();
      break;

    case 'p':
      streamEnabled = !streamEnabled;
      Serial.print(F("[STREAM] "));
      Serial.println(streamEnabled ? F("ON") : F("OFF"));
      break;

    case 'b':
      tone(BUZZER_PIN, 2200, 150);
      Serial.println(F("[BUZZER] short beep"));
      break;

    case 'l':
      runLedTest();
      updateHazardState(true);
      if (gHazardDetected)
      {
        setStatusLed(CRGB::Black);
        drawAlertOled(false);
      }
      else
      {
        setStatusLed(CRGB::White);
        drawNormalOled();
      }
      break;

    case 'v':
      runServoSweep();
      break;

    case 'c':
      centerServo();
      break;

    default:
      break;
  }
}

void dispatchJetsonLine(const char* line)
{
  // ── PERSON_DETECTED ────────────────────────────────────────
  if (strcmp(line, "PERSON_DETECTED") == 0)
  {
    Serial.println(F("[JETSON] PERSON_DETECTED"));
    if (gMode == MODE_PATROL)
    {
      enterVerification();
    }
  }

  // ── FACE_VERIFIED:<name> ───────────────────────────────────
  else if (strncmp(line, "FACE_VERIFIED:", 14) == 0)
  {
    const char* name = line + 14;
    Serial.print(F("[JETSON] FACE_VERIFIED: "));
    Serial.println(name);
    if (gMode == MODE_VERIFICATION)
    {
      noTone(BUZZER_PIN);
      setStatusLed(CRGB::Green);
      Serial.print(F("PLAY:verified_"));
      Serial.println(name);
      delay(1000);
      enterPatrol();
    }
  }

  // ── FACE_UNKNOWN ───────────────────────────────────────────
  else if (strcmp(line, "FACE_UNKNOWN") == 0)
  {
    Serial.println(F("[JETSON] FACE_UNKNOWN"));
    if (gMode == MODE_VERIFICATION)
    {
      enterSecurityAlert();
    }
  }

  // ── FACE_TIMEOUT ───────────────────────────────────────────
  else if (strcmp(line, "FACE_TIMEOUT") == 0)
  {
    Serial.println(F("[JETSON] FACE_TIMEOUT"));
    if (gMode == MODE_VERIFICATION)
    {
      enterSecurityAlert();
    }
  }

  // ── HEARTBEAT ──────────────────────────────────────────────
  else if (strcmp(line, "HEARTBEAT") == 0)
  {
    // silently acknowledge — uncomment next line to debug:
    // Serial.println(F("[JETSON] HEARTBEAT"));
  }

  // ── unknown ────────────────────────────────────────────────
  else
  {
    Serial.print(F("[SERIAL] unknown: "));
    Serial.println(line);
  }
}

void handleSerial()
{
  while (Serial.available() > 0)
  {
    const char c = (char)Serial.read();

    if (c == '\r') continue;  // ignore CR (Windows line endings)

    if (c == '\n')
    {
      sLineBuf[sLineLen] = '\0';

      if (sLineLen == 1)
      {
        // single interactive command — keep existing behaviour
        dispatchSingleChar((char)tolower((unsigned char)sLineBuf[0]));
      }
      else if (sLineLen > 1)
      {
        // multi-char line from Jetson
        dispatchJetsonLine(sLineBuf);
      }

      sLineLen = 0;
    }
    else
    {
      if (sLineLen < sizeof(sLineBuf) - 1)
      {
        sLineBuf[sLineLen++] = c;
      }
      // silently drop chars if buffer overflows
    }
  }
}

// ============================================================
// setup / loop
// ============================================================
void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial3.begin(BT_BAUD);   // HC-05 on TX3=D14, RX3=D15
  Serial.println(F("[BT] Serial3 ready"));

  for (size_t i = 0; i < (sizeof(kDualSensors) / sizeof(kDualSensors[0])); ++i)
  {
    pinMode(kDualSensors[i].digitalPin, INPUT);
    pinMode(kDualSensors[i].analogPin, INPUT);
  }

  for (size_t i = 0; i < (sizeof(kUltrasonicSensors) / sizeof(kUltrasonicSensors[0])); ++i)
  {
    pinMode(kUltrasonicSensors[i].trigPin, OUTPUT);
    pinMode(kUltrasonicSensors[i].echoPin, INPUT);
    digitalWrite(kUltrasonicSensors[i].trigPin, LOW);
  }

  setupMotorPins();
  stopMotors();

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(statusLeds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  setStatusLed(CRGB::White);

  testServo.attach(SERVO_PIN);
  centerServo();

  initOled();
  drawNormalOled();

  delay(300);
  runLedTest();
  setStatusLed(CRGB::White);
  drawNormalOled();

  updateHazardState(true);

  printHelp();
  printSnapshot();

  enterPatrol();  // start in patrol mode
}

void loop()
{
  handleSerial();
  updateHazardState(false);  // always poll sensors (rate-limited to 100 ms internally)
  runCurrentMode();          // mode-specific outputs, transitions, motor drive

  const unsigned long nowMs = millis();
  if (streamEnabled && (nowMs - lastStreamMs) >= STREAM_INTERVAL_MS)
  {
    lastStreamMs = nowMs;
    printSnapshot();
  }
}