#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET 28
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================= OLED / Servo =================
int pan = 90;
int tilt = 120;
int servo_min = 20;
int servo_max = 160;

Servo servo_pan;
Servo servo_tilt;

// ================= Serial =================
#define SERIAL   Serial
#define BTSERIAL Serial3

// ================= Motor pins =================
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

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW);  digitalWrite(DIRA2,HIGH); analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW);  digitalWrite(DIRA2,LOW);  analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH); digitalWrite(DIRA2,LOW);  analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW);  digitalWrite(DIRB2,HIGH); analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW);  digitalWrite(DIRB2,LOW);  analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,LOW);  analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW);  digitalWrite(DIRC2,HIGH); analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW);  digitalWrite(DIRC2,LOW);  analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH); digitalWrite(DIRC2,LOW);  analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW);  digitalWrite(DIRD2,HIGH); analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW);  digitalWrite(DIRD2,LOW);  analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,LOW);  analogWrite(PWMD,pwm);}while(0)

// ================= Ultrasonic pins =================
const int LEFT_TRIG  = 32;   // PC5
const int LEFT_ECHO  = 33;   // PC4
const int RIGHT_TRIG = 22;   // PA0
const int RIGHT_ECHO = 24;   // PA2

// ================= Settings =================
// Reduced speed.
const int BASE_FORWARD_PWM = 52;
const int BASE_BACK_PWM    = 52;
const int BASE_ROTATE_PWM  = 16;

// Drift correction.
// If the robot drifts left, the right side is usually stronger.
// Start by reducing the right motors (B, D) slightly.
const int TRIM_A = 0;    // left front
const int TRIM_B = -8;   // right front
const int TRIM_C = 0;    // left rear
const int TRIM_D = -8;   // right rear

const unsigned long AUTO_DELAY_MS = 2000;
const unsigned long FORWARD_MS    = 2200;
const unsigned long PRINT_MS      = 250;

unsigned long bootTime = 0;
unsigned long stateStartTime = 0;
unsigned long lastPrintTime = 0;

// ================= State =================
enum AutoState {
  WAIT_DELAY,
  INITIAL_FORWARD,
  DONE
};

AutoState autoState = WAIT_DELAY;

// ================= Utility =================
int clipPWM(int value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return value;
}

void setupMotorPins() {
  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);

  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);

  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);
}

void setupUltrasonicPins() {
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  digitalWrite(LEFT_TRIG, LOW);
  digitalWrite(RIGHT_TRIG, LOW);
}

void updateOLED(const char* line1, const char* line2 = "") {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(line1);
  if (strlen(line2) > 0) {
    display.println(line2);
  }
  display.display();
}

// ================= Ultrasonic =================
float readOneUS(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return -1.0;

  return duration * 0.0343f / 2.0f;
}

float readUSAvg(int trigPin, int echoPin) {
  float a = readOneUS(trigPin, echoPin);
  delay(8);
  float b = readOneUS(trigPin, echoPin);
  delay(8);
  float c = readOneUS(trigPin, echoPin);

  float sum = 0;
  int count = 0;

  if (a > 0) { sum += a; count++; }
  if (b > 0) { sum += b; count++; }
  if (c > 0) { sum += c; count++; }

  if (count == 0) return -1.0;
  return sum / count;
}

void printUltrasonic() {
  if (millis() - lastPrintTime < PRINT_MS) return;
  lastPrintTime = millis();

  float leftDist = readUSAvg(LEFT_TRIG, LEFT_ECHO);
  delay(60);
  float rightDist = readUSAvg(RIGHT_TRIG, RIGHT_ECHO);

  SERIAL.print("Left: ");
  SERIAL.print(leftDist);
  SERIAL.print(" cm   Right: ");
  SERIAL.print(rightDist);
  SERIAL.print(" cm   Diff: ");
  SERIAL.println(leftDist - rightDist);
}

// ================= Motion =================
void STOP()
{
  MOTORA_STOP(0);
  MOTORB_STOP(0);
  MOTORC_STOP(0);
  MOTORD_STOP(0);
}

// Trimmed forward drive.
void ADVANCE_TRIMMED()
{
  int pwmA = clipPWM(BASE_FORWARD_PWM + TRIM_A);
  int pwmB = clipPWM(BASE_FORWARD_PWM + TRIM_B);
  int pwmC = clipPWM(BASE_FORWARD_PWM + TRIM_C);
  int pwmD = clipPWM(BASE_FORWARD_PWM + TRIM_D);

  MOTORA_FORWARD(pwmA);
  MOTORB_BACKOFF(pwmB);
  MOTORC_FORWARD(pwmC);
  MOTORD_BACKOFF(pwmD);
}

// Trimmed reverse drive.
void BACK_TRIMMED()
{
  int pwmA = clipPWM(BASE_BACK_PWM + TRIM_A);
  int pwmB = clipPWM(BASE_BACK_PWM + TRIM_B);
  int pwmC = clipPWM(BASE_BACK_PWM + TRIM_C);
  int pwmD = clipPWM(BASE_BACK_PWM + TRIM_D);

  MOTORA_BACKOFF(pwmA);
  MOTORB_FORWARD(pwmB);
  MOTORC_BACKOFF(pwmC);
  MOTORD_FORWARD(pwmD);
}

// Slow rotation.
void turnLeftSlow()
{
  MOTORA_BACKOFF(BASE_ROTATE_PWM);
  MOTORB_BACKOFF(BASE_ROTATE_PWM);
  MOTORC_BACKOFF(BASE_ROTATE_PWM);
  MOTORD_BACKOFF(BASE_ROTATE_PWM);
}

void turnRightSlow()
{
  MOTORA_FORWARD(BASE_ROTATE_PWM);
  MOTORB_FORWARD(BASE_ROTATE_PWM);
  MOTORC_FORWARD(BASE_ROTATE_PWM);
  MOTORD_FORWARD(BASE_ROTATE_PWM);
}

// ================= Control =================
void autonomousControl() {
  switch (autoState) {
    case WAIT_DELAY:
      STOP();
      if (millis() - bootTime >= AUTO_DELAY_MS) {
        autoState = INITIAL_FORWARD;
        stateStartTime = millis();
        updateOLED("FORWARD", "TRIM TEST");
        SERIAL.println("STATE: INITIAL_FORWARD");
      }
      break;

    case INITIAL_FORWARD:
      ADVANCE_TRIMMED();
      if (millis() - stateStartTime >= FORWARD_MS) {
        STOP();
        autoState = DONE;
        updateOLED("DONE", "STOP");
        SERIAL.println("STATE: DONE");
      }
      break;

    case DONE:
      STOP();
      break;
  }
}

// ================= Setup =================
void setup()
{
  SERIAL.begin(115200);
  BTSERIAL.begin(9600);

  setupMotorPins();
  setupUltrasonicPins();

  servo_pan.attach(48);
  servo_tilt.attach(47);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    SERIAL.println(F("SSD1306 allocation failed"));
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.println("AI Robot");
  display.display();

  STOP();

  pan = constrain(pan, servo_min, servo_max);
  tilt = constrain(tilt, servo_min, servo_max);
  servo_pan.write(pan);
  servo_tilt.write(tilt);

  bootTime = millis();

  SERIAL.println("BOOT OK");
  SERIAL.println("Wait 2 sec -> trimmed forward -> stop");
  SERIAL.println("Current trim: B=-8, D=-8");
  updateOLED("BOOT OK", "WAIT 2 SEC");
}

// ================= Loop =================
void loop()
{
  pan = constrain(pan, servo_min, servo_max);
  tilt = constrain(tilt, servo_min, servo_max);
  servo_pan.write(pan);
  servo_tilt.write(tilt);

  autonomousControl();
  printUltrasonic();
}
