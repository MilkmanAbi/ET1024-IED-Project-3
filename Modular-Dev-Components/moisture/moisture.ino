const int moisturePin = 7;  // Sensor D0 connected to D7

void setup() {
  Serial.begin(115200);
  pinMode(moisturePin, INPUT);
}

void loop() {
  int state = digitalRead(moisturePin);

  Serial.print("Digital State: ");
  Serial.print(state);

  if (state == LOW) {
    Serial.println("  -> Moisture detected");
  } else {
    Serial.println("  -> Dry");
  }

  delay(500);
}
