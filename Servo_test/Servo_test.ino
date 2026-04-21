#include <Servo.h>

Servo myServo;
const int SERVO_PIN = 39;

const int DOWN_ANGLE = 90;
const int UP_ANGLE = 130;

const int UP_DELAY_MS = 80;    // for 'u'
const int DOWN_DELAY_MS = 40;  // for 'd'

void smoothMove(int fromAngle, int toAngle, int stepDelayMs) {
  if (fromAngle < toAngle) {
    for (int angle = fromAngle; angle <= toAngle; angle++) {
      myServo.write(angle);
      delay(stepDelayMs);
    }
  } else {
    for (int angle = fromAngle; angle >= toAngle; angle--) {
      myServo.write(angle);
      delay(stepDelayMs);
    }
  }
}

void setup() {
  Serial.begin(115200);
  myServo.attach(SERVO_PIN);
  myServo.write(DOWN_ANGLE);

  Serial.println("Servo control ready.");
  Serial.println("Type 'u' to move 90 -> 130 slowly");
  Serial.println("Type 'd' to move 130 -> 90 faster");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    while (Serial.available()) {
      Serial.read();
    }

    if (cmd == 'u' || cmd == 'U') {
      Serial.println("Moving up: 90 -> 130");
      smoothMove(DOWN_ANGLE, UP_ANGLE, UP_DELAY_MS);
      Serial.println("Done.");
    }
    else if (cmd == 'd' || cmd == 'D') {
      Serial.println("Moving down: 130 -> 90");
      smoothMove(UP_ANGLE, DOWN_ANGLE, DOWN_DELAY_MS);
      Serial.println("Done.");
    }
    else {
      Serial.println("Invalid command. Use 'u' or 'd'.");
    }
  }
}