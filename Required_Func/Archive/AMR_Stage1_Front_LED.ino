#include <Arduino.h>
#include <Servo.h>

// =====================================================
// Stage 1 only: home toward the first far-wall LED
// =====================================================
// This sketch follows the experimentally observed pattern:
// - front / far-wall target: S70 > S90 > S110 and S70 - S110 is positive
// - side-wall LED:           S70 < S90 < S110 and S70 - S110 is negative
//
// Important physical note:
// The only remaining on-floor uncertainty is the fine-steering sign.
// If the AMR rotates the wrong way when F50 or F90 wins, change
// F50_WIN_MEANS_ROTATE_HIGHER_PAN below.

// =====================================================
// Hardware mapping
// =====================================================
const int SENSOR_A_PIN = A0;
const int SENSOR_B_PIN = A2;
const int SERVO_PAN_PIN = 28;

Servo servo_pan;

// =====================================================
// Motor pins and drive macros (preserved AMR style)
// =====================================================
#define PWMA 12
#define DIRA1 34
#define DIRA2 35

#define PWMB 8
#define DIRB1 37
#define DIRB2 36

#define PWMC 9
#define DIRC1 43
#define DIRC2 42

#define PWMD 5
#define DIRD1 A4
#define DIRD2 A5

#define MOTORA_FORWARD(pwm) do { digitalWrite(DIRA1, LOW);  digitalWrite(DIRA2, HIGH); analogWrite(PWMA, pwm); } while (0)
#define MOTORA_STOP(x)      do { digitalWrite(DIRA1, LOW);  digitalWrite(DIRA2, LOW);  analogWrite(PWMA, 0);   } while (0)
#define MOTORA_BACKOFF(pwm) do { digitalWrite(DIRA1, HIGH); digitalWrite(DIRA2, LOW);  analogWrite(PWMA, pwm); } while (0)

#define MOTORB_FORWARD(pwm) do { digitalWrite(DIRB1, LOW);  digitalWrite(DIRB2, HIGH); analogWrite(PWMB, pwm); } while (0)
#define MOTORB_STOP(x)      do { digitalWrite(DIRB1, LOW);  digitalWrite(DIRB2, LOW);  analogWrite(PWMB, 0);   } while (0)
#define MOTORB_BACKOFF(pwm) do { digitalWrite(DIRB1, HIGH); digitalWrite(DIRB2, LOW);  analogWrite(PWMB, pwm); } while (0)

#define MOTORC_FORWARD(pwm) do { digitalWrite(DIRC1, LOW);  digitalWrite(DIRC2, HIGH); analogWrite(PWMC, pwm); } while (0)
#define MOTORC_STOP(x)      do { digitalWrite(DIRC1, LOW);  digitalWrite(DIRC2, LOW);  analogWrite(PWMC, 0);   } while (0)
#define MOTORC_BACKOFF(pwm) do { digitalWrite(DIRC1, HIGH); digitalWrite(DIRC2, LOW);  analogWrite(PWMC, pwm); } while (0)

#define MOTORD_FORWARD(pwm) do { digitalWrite(DIRD1, LOW);  digitalWrite(DIRD2, HIGH); analogWrite(PWMD, pwm); } while (0)
#define MOTORD_STOP(x)      do { digitalWrite(DIRD1, LOW);  digitalWrite(DIRD2, LOW);  analogWrite(PWMD, 0);   } while (0)
#define MOTORD_BACKOFF(pwm) do { digitalWrite(DIRD1, HIGH); digitalWrite(DIRD2, LOW);  analogWrite(PWMD, pwm); } while (0)

// =====================================================
// Tunables
// =====================================================
int Motor_PWM = 50;

const int PAN_MIN = 40;
const int PAN_MAX = 140;
const int PAN_NEUTRAL = 90;
const int PAN_LOCK = 70;

const int COARSE_PAN_1 = 70;
const int COARSE_PAN_2 = 90;
const int COARSE_PAN_3 = 110;

const int FINE_PAN_1 = 50;
const int FINE_PAN_2 = 70;
const int FINE_PAN_3 = 90;

const int SERVO_SETTLE_MS = 40;
const int SENSOR_SAMPLES = 10;
const int SENSOR_SAMPLE_DELAY_MS = 2;
const int POST_ACTION_STOP_MS = 90;

const int FRONT_SIGNATURE_THRESHOLD = 15;
const int COARSE_CONTRAST_MIN = 12;
const int COARSE_ORDER_TOLERANCE = 4;
const int FINE_MARGIN = 8;

const int BLIND_FORWARD_MS = 320;
const int TRACK_FORWARD_MS = 260;
const int ROTATE_BURST_MS = 110;

// Tune on the real robot after a few approach runs.
const int CENTER_REACHED_SUM = 700;

// Default assumption:
// - if F50 wins, target is on the lower-pan side, so rotate toward lower pan
// Flip to true if the real robot shows the opposite behavior.
const bool F50_WIN_MEANS_ROTATE_HIGHER_PAN = false;

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

struct CoarseScanResult
{
  SensorReading s70;
  SensorReading s90;
  SensorReading s110;
  int frontMetric;
  int coarseContrast;
  bool frontOrder;
  bool sideOrder;
  bool frontCandidate;
  bool sideLike;
};

enum SteeringDecision
{
  DECISION_BLIND_FORWARD,
  DECISION_TRACK_FORWARD,
  DECISION_ROTATE_HIGHER_PAN,
  DECISION_ROTATE_LOWER_PAN,
  DECISION_STAGE_COMPLETE
};

struct FineScanResult
{
  SensorReading f50;
  SensorReading f70;
  SensorReading f90;
  int maxSum;
  SteeringDecision decision;
};

// =====================================================
// Forward declarations
// =====================================================
void ADVANCE();
void BACK();
void rotate_1();
void rotate_2();
void STOP();
void rotateTowardHigherPan();
void rotateTowardLowerPan();
void forwardBurst(int ms);
void rotateHigherPanBurst(int ms);
void rotateLowerPanBurst(int ms);
void setupMotorPins();
int readAverageRaw(int pin, int samples = SENSOR_SAMPLES);
void calibrateDarkWithHand();
SensorReading sampleAtPan(int panAngle);
CoarseScanResult performCoarseScan();
FineScanResult performFineScan();
SteeringDecision chooseFineDecision(const FineScanResult &scan);
bool stageReachedTarget(const FineScanResult &scan);
void applyDecision(SteeringDecision decision);
void printSample(const __FlashStringHelper *label, const SensorReading &reading);
void printCoarseSummary(const CoarseScanResult &scan);
void printFineSummary(const FineScanResult &scan);
const __FlashStringHelper *coarseClassName(const CoarseScanResult &scan);
const __FlashStringHelper *decisionName(SteeringDecision decision);
int max3(int a, int b, int c);
int min3(int a, int b, int c);
bool approxGte(int lhs, int rhs, int tolerance);

// =====================================================
// Dark calibration values
// =====================================================
int darkA = 0;
int darkB = 0;

bool stage1Complete = false;

// =====================================================
// Motion primitives
// =====================================================
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}

void BACK()
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}

void rotate_1()
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}

void rotate_2()
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}

void STOP()
{
  MOTORA_STOP(0);
  MOTORB_STOP(0);
  MOTORC_STOP(0);
  MOTORD_STOP(0);
}

void rotateTowardHigherPan()
{
  rotate_2();
}

void rotateTowardLowerPan()
{
  rotate_1();
}

void forwardBurst(int ms)
{
  ADVANCE();
  delay(ms);
  STOP();
  delay(POST_ACTION_STOP_MS);
}

void rotateHigherPanBurst(int ms)
{
  rotateTowardHigherPan();
  delay(ms);
  STOP();
  delay(POST_ACTION_STOP_MS);
}

void rotateLowerPanBurst(int ms)
{
  rotateTowardLowerPan();
  delay(ms);
  STOP();
  delay(POST_ACTION_STOP_MS);
}

// =====================================================
// Setup helpers
// =====================================================
void setupMotorPins()
{
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(PWMD, OUTPUT);

  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);
}

// =====================================================
// Sensor helpers
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

void calibrateDarkWithHand()
{
  Serial.println(F("[CAL] Cover the full photoresistor module with your hand."));
  for (int seconds = 5; seconds >= 1; seconds--)
  {
    Serial.print(F("[CAL] Dark calibration in "));
    Serial.print(seconds);
    Serial.println(F("..."));
    delay(1000);
  }

  long totalA = 0;
  long totalB = 0;
  const int calibrationSamples = 80;

  Serial.println(F("[CAL] Sampling dark baseline now. Keep it covered."));
  for (int i = 0; i < calibrationSamples; i++)
  {
    totalA += analogRead(SENSOR_A_PIN);
    totalB += analogRead(SENSOR_B_PIN);
    delay(5);
  }

  darkA = (int)(totalA / calibrationSamples);
  darkB = (int)(totalB / calibrationSamples);

  Serial.print(F("[CAL] darkA="));
  Serial.print(darkA);
  Serial.print(F(" darkB="));
  Serial.println(darkB);

  Serial.println(F("[CAL] Remove your hand."));
  delay(1500);
}

SensorReading sampleAtPan(int panAngle)
{
  SensorReading reading;

  reading.pan = constrain(panAngle, PAN_MIN, PAN_MAX);
  servo_pan.write(reading.pan);
  delay(SERVO_SETTLE_MS);

  reading.rawA = readAverageRaw(SENSOR_A_PIN);
  reading.rawB = readAverageRaw(SENSOR_B_PIN);

  reading.corrA = reading.rawA - darkA;
  reading.corrB = reading.rawB - darkB;

  if (reading.corrA < 0) reading.corrA = 0;
  if (reading.corrB < 0) reading.corrB = 0;

  reading.sum = reading.corrA + reading.corrB;
  return reading;
}

// =====================================================
// Scan logic
// =====================================================
CoarseScanResult performCoarseScan()
{
  CoarseScanResult scan;

  scan.s70 = sampleAtPan(COARSE_PAN_1);
  printSample(F("COARSE_70"), scan.s70);

  scan.s90 = sampleAtPan(COARSE_PAN_2);
  printSample(F("COARSE_90"), scan.s90);

  scan.s110 = sampleAtPan(COARSE_PAN_3);
  printSample(F("COARSE_110"), scan.s110);

  scan.frontMetric = scan.s70.sum - scan.s110.sum;
  scan.coarseContrast = max3(scan.s70.sum, scan.s90.sum, scan.s110.sum) -
                        min3(scan.s70.sum, scan.s90.sum, scan.s110.sum);

  scan.frontOrder = approxGte(scan.s70.sum, scan.s90.sum, COARSE_ORDER_TOLERANCE) &&
                    approxGte(scan.s90.sum, scan.s110.sum, COARSE_ORDER_TOLERANCE);

  scan.sideOrder = approxGte(scan.s90.sum, scan.s70.sum, COARSE_ORDER_TOLERANCE) &&
                   approxGte(scan.s110.sum, scan.s90.sum, COARSE_ORDER_TOLERANCE);

  scan.frontCandidate = (scan.frontMetric >= FRONT_SIGNATURE_THRESHOLD) &&
                        (scan.coarseContrast >= COARSE_CONTRAST_MIN) &&
                        scan.frontOrder;

  scan.sideLike = (scan.frontMetric <= -FRONT_SIGNATURE_THRESHOLD) &&
                  (scan.coarseContrast >= COARSE_CONTRAST_MIN) &&
                  scan.sideOrder;

  return scan;
}

FineScanResult performFineScan()
{
  FineScanResult scan;

  scan.f50 = sampleAtPan(FINE_PAN_1);
  printSample(F("FINE_50"), scan.f50);

  scan.f70 = sampleAtPan(FINE_PAN_2);
  printSample(F("FINE_70"), scan.f70);

  scan.f90 = sampleAtPan(FINE_PAN_3);
  printSample(F("FINE_90"), scan.f90);

  scan.maxSum = max3(scan.f50.sum, scan.f70.sum, scan.f90.sum);
  scan.decision = chooseFineDecision(scan);
  return scan;
}

SteeringDecision chooseFineDecision(const FineScanResult &scan)
{
  const int otherMaxFor50 = (scan.f70.sum > scan.f90.sum) ? scan.f70.sum : scan.f90.sum;
  const int otherMaxFor90 = (scan.f50.sum > scan.f70.sum) ? scan.f50.sum : scan.f70.sum;

  if (scan.f50.sum >= otherMaxFor50 + FINE_MARGIN)
  {
    return F50_WIN_MEANS_ROTATE_HIGHER_PAN ? DECISION_ROTATE_HIGHER_PAN : DECISION_ROTATE_LOWER_PAN;
  }

  if (scan.f90.sum >= otherMaxFor90 + FINE_MARGIN)
  {
    return F50_WIN_MEANS_ROTATE_HIGHER_PAN ? DECISION_ROTATE_LOWER_PAN : DECISION_ROTATE_HIGHER_PAN;
  }

  return DECISION_TRACK_FORWARD;
}

bool stageReachedTarget(const FineScanResult &scan)
{
  return (scan.f70.sum >= CENTER_REACHED_SUM) && (scan.decision == DECISION_TRACK_FORWARD);
}

void applyDecision(SteeringDecision decision)
{
  Serial.print(F("[ACTION] decision="));
  Serial.println(decisionName(decision));

  switch (decision)
  {
    case DECISION_BLIND_FORWARD:
      servo_pan.write(PAN_NEUTRAL);
      delay(60);
      forwardBurst(BLIND_FORWARD_MS);
      break;

    case DECISION_TRACK_FORWARD:
      servo_pan.write(PAN_LOCK);
      delay(60);
      forwardBurst(TRACK_FORWARD_MS);
      break;

    case DECISION_ROTATE_HIGHER_PAN:
      servo_pan.write(PAN_LOCK);
      delay(60);
      rotateHigherPanBurst(ROTATE_BURST_MS);
      break;

    case DECISION_ROTATE_LOWER_PAN:
      servo_pan.write(PAN_LOCK);
      delay(60);
      rotateLowerPanBurst(ROTATE_BURST_MS);
      break;

    case DECISION_STAGE_COMPLETE:
      stage1Complete = true;
      STOP();
      servo_pan.write(PAN_NEUTRAL);
      break;
  }
}

// =====================================================
// Debug helpers
// =====================================================
void printSample(const __FlashStringHelper *label, const SensorReading &reading)
{
  Serial.print(F("[SAMPLE] label="));
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
  Serial.print(F(" sum="));
  Serial.println(reading.sum);
}

void printCoarseSummary(const CoarseScanResult &scan)
{
  Serial.println(F("[COARSE] ----------------"));
  Serial.print(F("[COARSE] S70="));
  Serial.print(scan.s70.sum);
  Serial.print(F(" S90="));
  Serial.print(scan.s90.sum);
  Serial.print(F(" S110="));
  Serial.println(scan.s110.sum);

  Serial.print(F("[COARSE] frontMetric="));
  Serial.print(scan.frontMetric);
  Serial.print(F(" coarseContrast="));
  Serial.print(scan.coarseContrast);
  Serial.print(F(" frontOrder="));
  Serial.print(scan.frontOrder ? F("true") : F("false"));
  Serial.print(F(" sideOrder="));
  Serial.print(scan.sideOrder ? F("true") : F("false"));
  Serial.print(F(" class="));
  Serial.println(coarseClassName(scan));
}

void printFineSummary(const FineScanResult &scan)
{
  Serial.println(F("[FINE] ------------------"));
  Serial.print(F("[FINE] F50="));
  Serial.print(scan.f50.sum);
  Serial.print(F(" F70="));
  Serial.print(scan.f70.sum);
  Serial.print(F(" F90="));
  Serial.println(scan.f90.sum);

  Serial.print(F("[FINE] maxSum="));
  Serial.print(scan.maxSum);
  Serial.print(F(" decision="));
  Serial.println(decisionName(scan.decision));
}

const __FlashStringHelper *coarseClassName(const CoarseScanResult &scan)
{
  if (scan.frontCandidate) return F("front_candidate");
  if (scan.sideLike) return F("side_wall_like");
  return F("ambiguous_or_weak");
}

const __FlashStringHelper *decisionName(SteeringDecision decision)
{
  switch (decision)
  {
    case DECISION_BLIND_FORWARD:       return F("blind_forward");
    case DECISION_TRACK_FORWARD:       return F("track_forward");
    case DECISION_ROTATE_HIGHER_PAN:   return F("rotate_higher_pan");
    case DECISION_ROTATE_LOWER_PAN:    return F("rotate_lower_pan");
    case DECISION_STAGE_COMPLETE:      return F("stage_complete");
    default:                           return F("unknown");
  }
}

// =====================================================
// Utility helpers
// =====================================================
int max3(int a, int b, int c)
{
  int m = a;
  if (b > m) m = b;
  if (c > m) m = c;
  return m;
}

int min3(int a, int b, int c)
{
  int m = a;
  if (b < m) m = b;
  if (c < m) m = c;
  return m;
}

bool approxGte(int lhs, int rhs, int tolerance)
{
  return lhs >= (rhs - tolerance);
}

// =====================================================
// Arduino setup / loop
// =====================================================
void setup()
{
  Serial.begin(115200);

  pinMode(SENSOR_A_PIN, INPUT);
  pinMode(SENSOR_B_PIN, INPUT);

  setupMotorPins();
  STOP();

  servo_pan.attach(SERVO_PAN_PIN);
  servo_pan.write(PAN_NEUTRAL);
  delay(1000);

  Serial.println(F("[BOOT] Stage 1 front LED homing start"));
  Serial.println(F("[BOOT] Sensor A = A0, Sensor B = A2"));
  Serial.println(F("[BOOT] Pan servo pin = 28"));
  Serial.println(F("[BOOT] Coarse scan = 70, 90, 110"));
  Serial.println(F("[BOOT] Fine scan = 50, 70, 90"));
  Serial.println(F("[BOOT] If body rotation direction is wrong, swap rotateTowardHigherPan() / rotateTowardLowerPan()."));
  Serial.println(F("[BOOT] If fine steering sign is wrong, flip F50_WIN_MEANS_ROTATE_HIGHER_PAN."));

  calibrateDarkWithHand();
}

void loop()
{
  if (stage1Complete)
  {
    STOP();
    delay(200);
    return;
  }

  STOP();
  Serial.println();
  Serial.println(F("[LOOP] ===== New Stage-1 cycle ====="));

  CoarseScanResult coarse = performCoarseScan();
  printCoarseSummary(coarse);

  if (!coarse.frontCandidate)
  {
    applyDecision(DECISION_BLIND_FORWARD);
    return;
  }

  FineScanResult fine = performFineScan();
  printFineSummary(fine);

  if (stageReachedTarget(fine))
  {
    Serial.println(F("[FINE] Reached target threshold while aligned."));
    applyDecision(DECISION_STAGE_COMPLETE);
    return;
  }

  applyDecision(fine.decision);
}
