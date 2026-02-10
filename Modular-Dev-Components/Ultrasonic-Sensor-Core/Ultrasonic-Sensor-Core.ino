// Pins
const int trigPin = 10;
const int echoPin = 11;

void setup() {
  Serial.begin(9600);  // Serial at 9600 baud
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  long duration;
  float distanceCm;

  // Send a 10 µs pulse to trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pulse duration
  duration = pulseIn(echoPin, HIGH);

  // Convert to distance in cm
  distanceCm = duration * 0.034 / 2;  // Sound speed: 343 m/s

  // Print distance
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  // Presence check (example: object closer than 50 cm)
  if (distanceCm > 0 && distanceCm < 50) {
    Serial.println("Object detected!");
  }

  delay(500);  // half-second delay between readings
}
