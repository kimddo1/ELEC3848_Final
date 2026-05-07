#include <Arduino.h>
#include <Servo.h>

// =====================================================
// LDR calibration sketch
// =====================================================
// Purpose:
// 1. Do hand-cover dark calibration for A0 / A2.
// 2. Let you place the robot directly in front of one LED.
// 3. Measure how different sensor A and sensor B are at the same pose.
// 4. Print suggested per-sensor scale factors for later use in Stage 1.
//
// Important note:
// The two photoresistors are mounted at 90 degrees, so one single scale
// factor may not perfectly equalize them at every pan angle. This sketch
// therefore supports both:
// - single-pose calibration at the current pan
// - a short multi-angle front profile

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
const int PAN_MIN = 40;
const int PAN_MAX = 140;
const int PAN_START = 90;

const int SERVO_SETTLE_MS = 250;
const int SENSOR_SAMPLES = 10;
const int SENSOR_SAMPLE_DELAY_MS = 2;

const int DARK_CAL_SAMPLES = 80;
const int FRONT_CAL_REPEATS = 12;
const int FRONT_CAL_MIN_SIGNAL = 20;

// Live stream pace for Serial Monitor.
// Increase this if the output is still too fast to read comfortably.
const unsigned long STREAM_INTERVAL_MS = 1000;

const int PROFILE_COUNT = 4;
const int PROFILE_PANS[PROFILE_COUNT] = {50, 70, 90, 110};

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
  int scaledA;
  int scaledB;
  int sumCorr;
  int sumScaled;
};

struct AggregateReading
{
  long rawA;
  long rawB;
  long corrA;
  long corrB;
  long scaledA;
  long scaledB;
  long sumCorr;
  long sumScaled;
  int samples;
};

// =====================================================
// Forward declarations
// =====================================================
int readAverageRaw(int pin, int samples = SENSOR_SAMPLES);
int applyScaleX1000(int value, int scaleX1000);
void movePan(int angle);
SensorReading takeReadingAtPan(int angle);
SensorReading averageReadingAtPan(int angle, int repeats);
void printReading(const __FlashStringHelper *label, const SensorReading &reading);
void printScaleSummary();
void printHelp();
void calibrateDarkWithHand();
bool calibrateFrontAtCurrentPan();
bool calibrateFrontProfile();

// =====================================================
// Global state
// =====================================================
int currentPan = PAN_START;
int darkA = 0;
int darkB = 0;

// Fixed-point scale factors for later use:
// scaled = corr * scaleX1000 / 1000
int scaleA_X1000 = 1000;
int scaleB_X1000 = 1000;

bool streamOn = true;
unsigned long lastStreamMs = 0;

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

int applyScaleX1000(int value, int scaleX1000)
{
  long scaled = (long)value * (long)scaleX1000;
  scaled /= 1000L;

  if (scaled < 0) return 0;
  if (scaled > 32767L) return 32767;
  return (int)scaled;
}

void movePan(int angle)
{
  currentPan = constrain(angle, PAN_MIN, PAN_MAX);
  servo_pan.write(currentPan);
  delay(SERVO_SETTLE_MS);
}

SensorReading takeReadingAtPan(int angle)
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

  reading.scaledA = applyScaleX1000(reading.corrA, scaleA_X1000);
  reading.scaledB = applyScaleX1000(reading.corrB, scaleB_X1000);
  reading.sumCorr = reading.corrA + reading.corrB;
  reading.sumScaled = reading.scaledA + reading.scaledB;

  return reading;
}

SensorReading averageReadingAtPan(int angle, int repeats)
{
  AggregateReading total = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  SensorReading average;

  for (int i = 0; i < repeats; i++)
  {
    SensorReading reading = takeReadingAtPan(angle);

    total.rawA += reading.rawA;
    total.rawB += reading.rawB;
    total.corrA += reading.corrA;
    total.corrB += reading.corrB;
    total.scaledA += reading.scaledA;
    total.scaledB += reading.scaledB;
    total.sumCorr += reading.sumCorr;
    total.sumScaled += reading.sumScaled;
    total.samples++;
  }

  average.pan = currentPan;
  average.rawA = (int)(total.rawA / total.samples);
  average.rawB = (int)(total.rawB / total.samples);
  average.corrA = (int)(total.corrA / total.samples);
  average.corrB = (int)(total.corrB / total.samples);
  average.scaledA = (int)(total.scaledA / total.samples);
  average.scaledB = (int)(total.scaledB / total.samples);
  average.sumCorr = (int)(total.sumCorr / total.samples);
  average.sumScaled = (int)(total.sumScaled / total.samples);

  return average;
}

void printReading(const __FlashStringHelper *label, const SensorReading &reading)
{
  Serial.print(F("[READ] label="));
  Serial.print(label);
  Serial.print(F(" pan="));
  Serial.print(reading.pan);

  Serial.print(F(" rawA="));
  Serial.print(reading.rawA);
  Serial.print(F(" rawB="));
  Serial.print(reading.rawB);

  Serial.print(F(" corrA="));
  Serial.print(reading.corrA);
  Serial.print(F(" corrB="));
  Serial.print(reading.corrB);

  Serial.print(F(" scaledA="));
  Serial.print(reading.scaledA);
  Serial.print(F(" scaledB="));
  Serial.print(reading.scaledB);

  Serial.print(F(" sumCorr="));
  Serial.print(reading.sumCorr);
  Serial.print(F(" sumScaled="));
  Serial.println(reading.sumScaled);
}

void printScaleSummary()
{
  Serial.print(F("[SCALE] darkA="));
  Serial.print(darkA);
  Serial.print(F(" darkB="));
  Serial.println(darkB);

  Serial.print(F("[SCALE] scaleA_X1000="));
  Serial.print(scaleA_X1000);
  Serial.print(F(" scaleB_X1000="));
  Serial.println(scaleB_X1000);

  Serial.println(F("[SCALE] Use later as:"));
  Serial.println(F("[SCALE] scaledA = corrA * scaleA_X1000 / 1000"));
  Serial.println(F("[SCALE] scaledB = corrB * scaleB_X1000 / 1000"));
}

void printHelp()
{
  Serial.println();
  Serial.println(F("Commands:"));
  Serial.println(F("  j / l : pan -2 / +2"));
  Serial.println(F("  u / o : pan -10 / +10"));
  Serial.println(F("  c     : pan = 90"));
  Serial.println(F("  1     : pan = 50"));
  Serial.println(F("  2     : pan = 70"));
  Serial.println(F("  3     : pan = 90"));
  Serial.println(F("  4     : pan = 110"));
  Serial.println(F("  p     : print one reading"));
  Serial.println(F("  s     : toggle streaming"));
  Serial.println(F("  r     : redo dark calibration"));
  Serial.println(F("  k     : front calibration at current pan"));
  Serial.println(F("  m     : front profile calibration at 50/70/90/110"));
  Serial.println(F("  x     : print current dark/scale values"));
  Serial.println();
}

void calibrateDarkWithHand()
{
  Serial.println(F("[DARK] Cover the whole photoresistor module with your hand."));
  Serial.println(F("[DARK] Dark calibration starts in 5 seconds..."));
  delay(5000);

  long totalA = 0;
  long totalB = 0;

  Serial.println(F("[DARK] Sampling now. Keep covering."));
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

bool calibrateFrontAtCurrentPan()
{
  Serial.println(F("[FRONT] Single-pose calibration starting."));
  Serial.println(F("[FRONT] Keep the robot directly facing one LED and do not move it."));

  SensorReading ref = averageReadingAtPan(currentPan, FRONT_CAL_REPEATS);
  printReading(F("FRONT_SINGLE"), ref);

  if (ref.corrA < FRONT_CAL_MIN_SIGNAL || ref.corrB < FRONT_CAL_MIN_SIGNAL)
  {
    Serial.println(F("[FRONT] Signal too weak for useful gain calibration."));
    return false;
  }

  long target = (long)ref.corrA + (long)ref.corrB;
  target /= 2L;

  scaleA_X1000 = (int)((target * 1000L) / ref.corrA);
  scaleB_X1000 = (int)((target * 1000L) / ref.corrB);

  Serial.print(F("[FRONT] ratio A/B = "));
  Serial.println((float)ref.corrA / (float)ref.corrB, 4);
  printScaleSummary();

  SensorReading preview = averageReadingAtPan(currentPan, FRONT_CAL_REPEATS);
  printReading(F("FRONT_SINGLE_SCALED"), preview);

  return true;
}

bool calibrateFrontProfile()
{
  long totalCorrA = 0;
  long totalCorrB = 0;
  int validCount = 0;

  Serial.println(F("[PROFILE] Front-profile calibration starting."));
  Serial.println(F("[PROFILE] Keep robot fixed directly in front of one LED."));

  for (int i = 0; i < PROFILE_COUNT; i++)
  {
    SensorReading ref = averageReadingAtPan(PROFILE_PANS[i], FRONT_CAL_REPEATS);
    printReading(F("PROFILE"), ref);

    if (ref.corrA >= FRONT_CAL_MIN_SIGNAL && ref.corrB >= FRONT_CAL_MIN_SIGNAL)
    {
      totalCorrA += ref.corrA;
      totalCorrB += ref.corrB;
      validCount++;

      Serial.print(F("[PROFILE] pan="));
      Serial.print(ref.pan);
      Serial.print(F(" ratio A/B="));
      Serial.println((float)ref.corrA / (float)ref.corrB, 4);
    }
    else
    {
      Serial.print(F("[PROFILE] pan="));
      Serial.print(ref.pan);
      Serial.println(F(" skipped: weak signal"));
    }
  }

  if (validCount == 0 || totalCorrA <= 0 || totalCorrB <= 0)
  {
    Serial.println(F("[PROFILE] No usable samples for calibration."));
    return false;
  }

  long avgCorrA = totalCorrA / validCount;
  long avgCorrB = totalCorrB / validCount;
  long target = (avgCorrA + avgCorrB) / 2L;

  scaleA_X1000 = (int)((target * 1000L) / avgCorrA);
  scaleB_X1000 = (int)((target * 1000L) / avgCorrB);

  Serial.print(F("[PROFILE] avgCorrA="));
  Serial.print(avgCorrA);
  Serial.print(F(" avgCorrB="));
  Serial.println(avgCorrB);

  Serial.print(F("[PROFILE] overall ratio A/B="));
  Serial.println((float)avgCorrA / (float)avgCorrB, 4);
  printScaleSummary();

  Serial.println(F("[PROFILE] Preview after applying scales:"));
  for (int i = 0; i < PROFILE_COUNT; i++)
  {
    SensorReading preview = averageReadingAtPan(PROFILE_PANS[i], FRONT_CAL_REPEATS);
    printReading(F("PROFILE_SCALED"), preview);
  }

  movePan(PAN_START);
  return true;
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
  movePan(PAN_START);

  Serial.println(F("LDR calibration sketch start"));
  Serial.println(F("Sensor A = A0, Sensor B = A2"));
  Serial.println(F("Pan servo pin = 28"));
  Serial.println(F("Do dark calibration first, then place robot directly in front of one LED."));

  calibrateDarkWithHand();
  printScaleSummary();
  printHelp();

  SensorReading first = takeReadingAtPan(currentPan);
  printReading(F("START"), first);
}

void loop()
{
  if (Serial.available())
  {
    char cmd = Serial.read();

    if (cmd == 'j')
    {
      movePan(currentPan - 2);
      printReading(F("PAN_STEP"), takeReadingAtPan(currentPan));
    }
    else if (cmd == 'l')
    {
      movePan(currentPan + 2);
      printReading(F("PAN_STEP"), takeReadingAtPan(currentPan));
    }
    else if (cmd == 'u')
    {
      movePan(currentPan - 10);
      printReading(F("PAN_STEP"), takeReadingAtPan(currentPan));
    }
    else if (cmd == 'o')
    {
      movePan(currentPan + 10);
      printReading(F("PAN_STEP"), takeReadingAtPan(currentPan));
    }
    else if (cmd == 'c' || cmd == '3')
    {
      movePan(90);
      printReading(F("PAN_SET"), takeReadingAtPan(currentPan));
    }
    else if (cmd == '1')
    {
      movePan(50);
      printReading(F("PAN_SET"), takeReadingAtPan(currentPan));
    }
    else if (cmd == '2')
    {
      movePan(70);
      printReading(F("PAN_SET"), takeReadingAtPan(currentPan));
    }
    else if (cmd == '4')
    {
      movePan(110);
      printReading(F("PAN_SET"), takeReadingAtPan(currentPan));
    }
    else if (cmd == 'p')
    {
      printReading(F("MANUAL"), takeReadingAtPan(currentPan));
    }
    else if (cmd == 's')
    {
      streamOn = !streamOn;
      Serial.print(F("[STREAM] streamOn="));
      Serial.println(streamOn ? F("true") : F("false"));
    }
    else if (cmd == 'r')
    {
      calibrateDarkWithHand();
      printScaleSummary();
      printReading(F("AFTER_DARK"), takeReadingAtPan(currentPan));
    }
    else if (cmd == 'k')
    {
      calibrateFrontAtCurrentPan();
    }
    else if (cmd == 'm')
    {
      calibrateFrontProfile();
    }
    else if (cmd == 'x')
    {
      printScaleSummary();
    }
    else if (cmd == 'h')
    {
      printHelp();
    }
  }

  if (streamOn && millis() - lastStreamMs >= STREAM_INTERVAL_MS)
  {
    lastStreamMs = millis();
    printReading(F("STREAM"), takeReadingAtPan(currentPan));
  }
}
