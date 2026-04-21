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
const int FRONT_ACTIVE_TH      = 80;    // IR_5_x: higher = active
const int FRONT_STRONG_TH      = 400;   // IR_5_x: higher = strong
const int GAS_THRESHOLD        = 150;    // MQ-2 analog threshold
const int GAS_STABLE_COUNT     = 3;

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
// Globals
// ============================================================
bool streamEnabled = true;
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
bool gHazardDetected = false;
const char* gDirection = "NONE";
const char* gHazardType = "NONE";

// ============================================================
// Function declarations
// ============================================================
int readAnalogAverage(uint8_t analogPin, int samples = ANALOG_SAMPLES);
float readDistanceCm(uint8_t trigPin, uint8_t echoPin);
float readDistanceMedian(uint8_t trigPin, uint8_t echoPin);

void setStatusLed(const CRGB& color);
void runLedTest();
void centerServo();
void runServoSweep();
void printHelp();
void printSnapshot();
void handleSerial();

bool sideFlameTriggered(int value);
bool frontActive(int value);
bool frontStrong(int value);

void readAllHazardSensors();
const char* getFrontDirectionFromCurrentReadings();
const char* getOverallDirectionFromCurrentReadings();
const char* getHazardTypeFromState(bool hazard, bool gasAlert, const char* direction);
void updateHazardState(bool forceRead = false);

bool isI2cPresent(uint8_t address);
bool beginOledAtAddress(uint8_t address);
void initOled();
void drawNormalOled();
void drawAlertOled(bool visible);

void updateAlertOutputs();

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
  gDirection = getOverallDirectionFromCurrentReadings();
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

// ============================================================
// LED + OLED fast alert update
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

void handleSerial()
{
  while (Serial.available() > 0)
  {
    const char command = (char)tolower((unsigned char)Serial.read());

    switch (command)
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
}

// ============================================================
// setup / loop
// ============================================================
void setup()
{
  Serial.begin(SERIAL_BAUD);

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
}

void loop()
{
  handleSerial();
  updateAlertOutputs();

  const unsigned long nowMs = millis();
  if (streamEnabled && (nowMs - lastStreamMs) >= STREAM_INTERVAL_MS)
  {
    lastStreamMs = nowMs;
    printSnapshot();
  }
}