#include <Servo.h>

// --- Pins from your working code ---
const int trigPin = 10;
const int echoPin = 11;
const int servoPin = 2; // Servo on D2

Servo myServo;

void setup() {
  Serial.begin(9600);  // Serial at 9600 baud
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  myServo.attach(servoPin);
  myServo.write(90); // Center the servo
  delay(1000);
}

void loop() {
  // Sweep from 30 to 150 (60 degrees left and right of center)
  for (int angle = 30; angle <= 150; angle++) {
    moveAndSense(angle);
  }
  // Sweep back from 150 to 30
  for (int angle = 150; angle >= 30; angle--) {
    moveAndSense(angle);
  }
}

void moveAndSense(int angle) {
  myServo.write(angle);
  delay(15); // Slightly faster sweep speed

  long duration;
  float distanceCm;

  // --- 1-for-1 Ultrasonic Logic from your file ---
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

  // Print results
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" | Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  // Presence check
  if (distanceCm > 0 && distanceCm < 50) {
    Serial.println("Object detected!");
  }
}
