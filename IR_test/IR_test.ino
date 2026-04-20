const int FLAME_ANALOG_PIN = A0;
const int FLAME_DIGITAL_PIN = 2;

void setup() {
  Serial.begin(9600);
  pinMode(FLAME_DIGITAL_PIN, INPUT);
}

void loop() {
  int analogValue = analogRead(FLAME_ANALOG_PIN);
  int digitalValue = digitalRead(FLAME_DIGITAL_PIN);

  Serial.print("Analog: ");
  Serial.print(analogValue);
  Serial.print(" | Digital: ");
  Serial.print(digitalValue);

  if (digitalValue == LOW) {
    Serial.println(" | Flame detected");
  } else {
    Serial.println(" | No flame");
  }

  delay(200);
}