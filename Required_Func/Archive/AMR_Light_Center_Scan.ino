#include <Arduino.h>
#include <Servo.h>

// =====================================================
// Servo + LDR scan-to-brightest-light test
// =====================================================
// Goal:
// - scan about 120 degrees with the pan servo
// - compute sum = correctedA + correctedB at each angle
// - remember the maximum sum point
// - turn the servo to that best angle
//
// This is a standalone test sketch for the updated sensor geometry
// where the two photoresistors are only about 20~30 degrees apart.

// =====================================================
// Hardware mapping
// =====================================================
const int SERVO_PAN_PIN = 28;
const int SENSOR_A_PIN = A0;
const int SENSOR_B_PIN = A2;

Servo servo_pan;

// =====================================================
// Tunables
// =====================================================
const int PAN_MIN = 30;
const int PAN_MAX = 150;
const int PAN_CENTER = 90;

const int SCAN_STEP_DEG = 4;
const int SERVO_SETTLE_MS = 120;
const int SENSOR_SAMPLES = 8;
const int SENSOR_SAMPLE_DELAY_MS = 2;

const int DARK_CAL_SAMPLES = 80;
const unsigned long AUTO_TRACK_INTERVAL_MS = 1800;

// =====================================================
// Data structures
// =====================================================
struct SensorReading
{
  int pan;
  int rawA;
  int rawB;
  int corrA;
  int corrB;
  int sum;
};

struct ScanResult
{
  int bestAngle;
  int bestSum;
  SensorReading bestReading;
};

// =====================================================
// Forward declarations
// =====================================================
int readAverageRaw(int pin, int samples = SENSOR_SAMPLES);
void movePan(int angle);
SensorReading sampleAtPan(int angle);
void printReading(const __FlashStringHelper *label, const SensorReading &reading);
void calibrateDarkWithHand();
ScanResult runFullScanAndLock();
void printHelp();

// =====================================================
// Global state
// =====================================================
int currentPan = PAN_CENTER;
int darkA = 0;
int darkB = 0;

bool autoTrackOn = false;
unsigned long lastAutoTrackMs = 0;

// =====================================================
// Helpers
// =====================================================
int readAverageRaw(int pin, int samples)
{
  long total = 0;
  for (int i = 0; i < samples; i++)
  {
    total += analogRead(pin);
    delay(SENSOR_SAMPLE_DELAY_MS);
  }
  return (int)(total / samples);
}

void movePan(int angle)
{
  currentPan = constrain(angle, PAN_MIN, PAN_MAX);
  servo_pan.write(currentPan);
  delay(SERVO_SETTLE_MS);
}

SensorReading sampleAtPan(int angle)
{
  SensorReading reading;
  movePan(angle);

  reading.pan = currentPan;
  reading.rawA = readAverageRaw(SENSOR_A_PIN);
  reading.rawB = readAverageRaw(SENSOR_B_PIN);

  reading.corrA = reading.rawA - darkA;
  reading.corrB = reading.rawB - darkB;
  if (reading.corrA < 0) reading.corrA = 0;
  if (reading.corrB < 0) reading.corrB = 0;

  reading.sum = reading.corrA + reading.corrB;
  return reading;
}

void printReading(const __FlashStringHelper *label, const SensorReading &reading)
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

  Serial.print(F(" sum="));
  Serial.println(reading.sum);
}

void calibrateDarkWithHand()
{
  Serial.println(F("[DARK] Cover the whole sensor module with your hand."));
  Serial.println(F("[DARK] Dark calibration starts in 5 seconds..."));
  delay(5000);

  long totalA = 0;
  long totalB = 0;
  for (int i = 0; i < DARK_CAL_SAMPLES; i++)
  {
    totalA += analogRead(SENSOR_A_PIN);
    totalB += analogRead(SENSOR_B_PIN);
    delay(5);
  }

  darkA = (int)(totalA / DARK_CAL_SAMPLES);
  darkB = (int)(totalB / DARK_CAL_SAMPLES);

  Serial.print(F("[DARK] darkA="));
  Serial.print(darkA);
  Serial.print(F(" darkB="));
  Serial.println(darkB);

  Serial.println(F("[DARK] Remove your hand."));
  delay(1500);
}

ScanResult runFullScanAndLock()
{
  ScanResult result;
  result.bestAngle = PAN_CENTER;
  result.bestSum = -1;

  Serial.println(F("[SCAN] ===== Start full scan ====="));

  for (int angle = PAN_MIN; angle <= PAN_MAX; angle += SCAN_STEP_DEG)
  {
    SensorReading reading = sampleAtPan(angle);
    printReading(F("SCAN"), reading);

    if (reading.sum > result.bestSum)
    {
      result.bestSum = reading.sum;
      result.bestAngle = reading.pan;
      result.bestReading = reading;
    }
  }

  Serial.print(F("[SCAN] Best angle = "));
  Serial.print(result.bestAngle);
  Serial.print(F(" best sum = "));
  Serial.println(result.bestSum);

  movePan(result.bestAngle);
  Serial.println(F("[LOCK] Servo moved to best angle."));
  printReading(F("LOCK"), sampleAtPan(result.bestAngle));

  return result;
}

void printHelp()
{
  Serial.println();
  Serial.println(F("Commands:"));
  Serial.println(F("  g : run full 120-degree scan and lock"));
  Serial.println(F("  t : toggle auto-track on/off"));
  Serial.println(F("  c : center servo"));
  Serial.println(F("  p : print current reading"));
  Serial.println(F("  r : redo dark calibration"));
  Serial.println(F("  a : pan -10"));
  Serial.println(F("  d : pan +10"));
  Serial.println(F("  j : pan -2"));
  Serial.println(F("  l : pan +2"));
  Serial.println();
}

// =====================================================
// Arduino setup / loop
// =====================================================
void setup()
{
  Serial.begin(115200);

  pinMode(SENSOR_A_PIN, INPUT);
  pinMode(SENSOR_B_PIN, INPUT);

  servo_pan.attach(SERVO_PAN_PIN);
  movePan(PAN_CENTER);

  Serial.println(F("Light-center scan test start"));
  Serial.println(F("Sensor A = A0, Sensor B = A2"));
  Serial.println(F("Pan servo pin = 28"));
  Serial.println(F("Scan range = 30 to 150 degrees"));
  Serial.println(F("Scan logic = max(correctedA + correctedB)"));

  calibrateDarkWithHand();
  printHelp();
  printReading(F("START"), sampleAtPan(currentPan));
}

void loop()
{
  if (Serial.available())
  {
    char cmd = Serial.read();

    if (cmd == 'g')
    {
      runFullScanAndLock();
    }
    else if (cmd == 't')
    {
      autoTrackOn = !autoTrackOn;
      Serial.print(F("[AUTO] autoTrackOn="));
      Serial.println(autoTrackOn ? F("true") : F("false"));
    }
    else if (cmd == 'c')
    {
      movePan(PAN_CENTER);
      printReading(F("CENTER"), sampleAtPan(currentPan));
    }
    else if (cmd == 'p')
    {
      printReading(F("MANUAL"), sampleAtPan(currentPan));
    }
    else if (cmd == 'r')
    {
      calibrateDarkWithHand();
      printReading(F("AFTER_DARK"), sampleAtPan(currentPan));
    }
    else if (cmd == 'a')
    {
      movePan(currentPan - 10);
      printReading(F("PAN_STEP"), sampleAtPan(currentPan));
    }
    else if (cmd == 'd')
    {
      movePan(currentPan + 10);
      printReading(F("PAN_STEP"), sampleAtPan(currentPan));
    }
    else if (cmd == 'j')
    {
      movePan(currentPan - 2);
      printReading(F("PAN_STEP"), sampleAtPan(currentPan));
    }
    else if (cmd == 'l')
    {
      movePan(currentPan + 2);
      printReading(F("PAN_STEP"), sampleAtPan(currentPan));
    }
  }

  if (autoTrackOn && millis() - lastAutoTrackMs >= AUTO_TRACK_INTERVAL_MS)
  {
    lastAutoTrackMs = millis();
    runFullScanAndLock();
  }
}
