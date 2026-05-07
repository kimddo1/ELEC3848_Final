#include <Arduino.h>
#include <Servo.h>

// ============================================================
// Live sensor monitor
// ============================================================
// Purpose:
// 1. Stream the robot's direct sensor readings to Serial Monitor in real time.
// 2. Let you move the pan servo manually from Serial commands.
// 3. Keep the monitor aligned with the preset LDR settings used in AMR_Final_V1.
//
// Hardware used:
// - Pan servo on pin 28
// - Photoresistors on A0 / A2
// - Front ultrasonic pair on 32/33 and 22/24
// - Side ultrasonic pair on PL5/PL4 and PA7/PA3

// ============================================================
// Pin Definitions
// ============================================================
const unsigned long SERIAL_BAUD = 115200UL;

const uint8_t SERVO_PIN = 28;

const uint8_t LDR_A_PIN = A0;
const uint8_t LDR_B_PIN = A2;

const uint8_t US_L_TRIG = 32;
const uint8_t US_L_ECHO = 33;
const uint8_t US_R_TRIG = 22;
const uint8_t US_R_ECHO = 24;

// Arduino Mega 2560 pin mapping:
// PL5 -> D44
// PL4 -> D45
// PA7 -> D29
// PA3 -> D25
const uint8_t SIDE_L_TRIG = 44;
const uint8_t SIDE_L_ECHO = 45;
const uint8_t SIDE_R_TRIG = 29;
const uint8_t SIDE_R_ECHO = 25;

// ============================================================
// Tunables
// ============================================================
const int SERVO_MIN = 30;
const int SERVO_MAX = 150;
const int SERVO_CENTER = 90;
const int SERVO_STEP = 5;
const int SERVO_SETTLE_MS = 180;

const int LDR_SAMPLES = 8;
const int LDR_SAMPLE_DELAY_MS = 2;

const int PRESET_DARK_A = 197;
const int PRESET_DARK_B = 188;
const int PRESET_AMBIENT_A = PRESET_DARK_A + 250;
const int PRESET_AMBIENT_B = PRESET_DARK_B + 250;
const float PRESET_GAIN_A = 1.00f;
const float PRESET_GAIN_B = 1.00f;
const float PRESET_LDR_TRIM = 0.0f;

const int ULTRA_TIMEOUT_US = 25000;
const int ULTRA_VALID_MIN_CM = 2;
const int ULTRA_VALID_MAX_CM = 400;

const unsigned long STREAM_INTERVAL_MS = 400UL;

// ============================================================
// Data Structures
// ============================================================
struct LdrReading
{
  int pan;
  int rawA;
  int rawB;
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

// ============================================================
// Globals
// ============================================================
Servo trackerServo;

int servoPos = SERVO_CENTER;
int darkA = 0;
int darkB = 0;
int ambientA = 0;
int ambientB = 0;
float gainA = PRESET_GAIN_A;
float gainB = PRESET_GAIN_B;
float ldrTrim = PRESET_LDR_TRIM;

bool streamOn = true;
unsigned long lastStreamMs = 0;

// ============================================================
// Forward Declarations
// ============================================================
void writeServoAngle(int angle);
void centerServo();
int readAverage(int pin, int samples = LDR_SAMPLES);
bool isDistanceValid(float cm);
float median3(float a, float b, float c);
float readDistanceCm(uint8_t trigPin, uint8_t echoPin);
float readDistanceMedian(uint8_t trigPin, uint8_t echoPin);
UltraPair readUltrasonicPair();
LdrReading readLdrNow();
void loadPresetLdrSettings();
void printHelp();
void printStatusOnce();
void handleSerial();

// ============================================================
// Helpers
// ============================================================
void writeServoAngle(int angle)
{
  servoPos = constrain(angle, SERVO_MIN, SERVO_MAX);
  trackerServo.write(servoPos);
  delay(SERVO_SETTLE_MS);
}

void centerServo()
{
  writeServoAngle(SERVO_CENTER);
}

int readAverage(int pin, int samples)
{
  long total = 0;
  for (int i = 0; i < samples; ++i)
  {
    total += analogRead(pin);
    delay(LDR_SAMPLE_DELAY_MS);
  }
  return (int)(total / samples);
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

float readDistanceMedian(uint8_t trigPin, uint8_t echoPin)
{
  float a = readDistanceCm(trigPin, echoPin);
  delay(8);
  float b = readDistanceCm(trigPin, echoPin);
  delay(8);
  float c = readDistanceCm(trigPin, echoPin);
  return median3(a, b, c);
}

UltraPair readUltrasonicPair()
{
  UltraPair pair;

  pair.leftCm = readDistanceMedian(US_L_TRIG, US_L_ECHO);
  delay(12);
  pair.rightCm = readDistanceMedian(US_R_TRIG, US_R_ECHO);
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

LdrReading readLdrNow()
{
  LdrReading reading;
  reading.pan = servoPos;
  reading.rawA = readAverage(LDR_A_PIN);
  reading.rawB = readAverage(LDR_B_PIN);
  return reading;
}

void loadPresetLdrSettings()
{
  darkA = PRESET_DARK_A;
  darkB = PRESET_DARK_B;
  ambientA = PRESET_AMBIENT_A;
  ambientB = PRESET_AMBIENT_B;
  gainA = PRESET_GAIN_A;
  gainB = PRESET_GAIN_B;
  ldrTrim = PRESET_LDR_TRIM;

  Serial.print(F("[PRESET] darkA="));
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

void printHelp()
{
  Serial.println();
  Serial.println(F("=== Live Readings Monitor ==="));
  Serial.println(F("T : toggle live stream"));
  Serial.println(F("P : print one reading now"));
  Serial.println(F("C : center servo"));
  Serial.println(F("H or ? : help"));
  Serial.println();
}

void printStatusOnce()
{
  if (servoPos != SERVO_CENTER)
  {
    centerServo();
  }

  LdrReading ldr = readLdrNow();
  UltraPair frontUltra = readUltrasonicPair();
  float sideLeftCm = readDistanceMedian(SIDE_L_TRIG, SIDE_L_ECHO);
  delay(12);
  float sideRightCm = readDistanceMedian(SIDE_R_TRIG, SIDE_R_ECHO);

  Serial.print(F("[LIVE] pan="));
  Serial.print(ldr.pan);

  Serial.print(F(" rawA="));
  Serial.print(ldr.rawA);
  Serial.print(F(" rawB="));
  Serial.print(ldr.rawB);

  Serial.print(F(" frontLeft="));
  Serial.print(frontUltra.leftCm, 1);

  Serial.print(F(" frontRight="));
  Serial.print(frontUltra.rightCm, 1);

  Serial.print(F(" sideLeft="));
  Serial.print(sideLeftCm, 1);

  Serial.print(F(" sideRight="));
  Serial.println(sideRightCm, 1);
}

void handleSerial()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') continue;
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');

    switch (c)
    {
      case 'T':
        streamOn = !streamOn;
        Serial.print(F("[STREAM] "));
        Serial.println(streamOn ? F("ON") : F("OFF"));
        break;

      case 'P':
        printStatusOnce();
        break;

      case 'C':
        centerServo();
        Serial.print(F("[SERVO] pan="));
        Serial.println(servoPos);
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
void setup()
{
  Serial.begin(SERIAL_BAUD);

  pinMode(US_L_TRIG, OUTPUT);
  pinMode(US_L_ECHO, INPUT);
  pinMode(US_R_TRIG, OUTPUT);
  pinMode(US_R_ECHO, INPUT);
  pinMode(SIDE_L_TRIG, OUTPUT);
  pinMode(SIDE_L_ECHO, INPUT);
  pinMode(SIDE_R_TRIG, OUTPUT);
  pinMode(SIDE_R_ECHO, INPUT);
  digitalWrite(US_L_TRIG, LOW);
  digitalWrite(US_R_TRIG, LOW);
  digitalWrite(SIDE_L_TRIG, LOW);
  digitalWrite(SIDE_R_TRIG, LOW);

  pinMode(LDR_A_PIN, INPUT);
  pinMode(LDR_B_PIN, INPUT);

  trackerServo.attach(SERVO_PIN);
  centerServo();

  Serial.println(F("ELEC3848 live readings monitor"));
  Serial.println(F("Front ultras: 32/33 and 22/24"));
  Serial.println(F("Side ultras: left 44/45 and right 29/25"));
  Serial.println(F("Servo starts centered at 90 deg for live LDR readings."));
  loadPresetLdrSettings();
  printHelp();
}

void loop()
{
  handleSerial();

  if (streamOn && (millis() - lastStreamMs >= STREAM_INTERVAL_MS))
  {
    lastStreamMs = millis();
    printStatusOnce();
  }
}
