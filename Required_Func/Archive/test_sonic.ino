// Simple ultrasonic sensor test
// Trig pin: 13
// Echo pin: 15

const int trigPin = 13;
const int echoPin = 15;

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, LOW);
}

void loop() {
  long duration;
  float distanceCm;

  // Send 10us trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo pulse duration
  duration = pulseIn(echoPin, HIGH);

  // Convert to distance in cm
  distanceCm = duration * 0.0343 / 2;

  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  delay(500);
}