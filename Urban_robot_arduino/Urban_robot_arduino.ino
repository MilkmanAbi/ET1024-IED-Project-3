/*
 * Urban Farming Robot - IED Project 3
 * Mobile Monitoring System for Climate-Responsive Environments
 * Challenge: 6G&AIoT Hub - Environmental Monitoring for Smart Farming
 * 
 * REQUIRED LIBRARIES TO INSTALL:
 * 1. LiquidCrystal_I2C (Version 1.1.2)
 *    Go to Arduino.cc -> search 'LiquidCrystal I2C'
 *    Download "LiquidCrystal_I2C-1.1.2.zip"
 *    Arduino IDE -> Sketch -> Include Library -> Add .ZIP Library
 * 
 * 2. DHT sensor library (by Adafruit)
 *    Arduino IDE -> Sketch -> Include Library -> Manage Libraries
 *    Search "DHT sensor library" -> Install
 * 
 * 3. Adafruit Unified Sensor
 *    Arduino IDE -> Sketch -> Include Library -> Manage Libraries
 *    Search "Adafruit Unified Sensor" -> Install
 */

// ============================================
// LIBRARY INCLUDES
// ============================================
#include <Wire.h>                    // I2C communication for LCD
#include <LiquidCrystal_I2C.h>       // LCD Display library
#include <DHT.h>                     // DHT11 sensor library

// ============================================
// PIN DEFINITIONS - IED ARDUINO SHIELD
// Based on IED Shield Schematic
// ============================================

// Motor Driver Pins (MX1508 on Shield)
// Motor A (Left Motor)
#define MOTOR_A_IN1      9     // PWM - Left motor speed control
#define MOTOR_A_IN2      6     // Direction - Left motor direction
// Motor B (Right Motor)
#define MOTOR_B_IN3      5     // PWM - Right motor speed control
#define MOTOR_B_IN4      3     // Direction - Right motor direction

// Ultrasonic Sensor (HC-SR04) - Connector P3
#define ULTRASONIC_TRIG  11    // Trigger pin (D11)
#define ULTRASONIC_ECHO  10    // Echo pin (D10)

// IR Sensors - For Line Tracking
#define IR_SENSOR_1      A0    // IR Sensor 1 - Connector P5 (Left sensor)
#define IR_SENSOR_2      A1    // IR Sensor 2 - Connector P6 (Right sensor)

// DHT11 Temperature & Humidity Sensor
#define DHT_PIN          12    // Digital pin for DHT11
#define DHT_TYPE         DHT11

// Moisture Sensor (Analog)
#define MOISTURE_PIN     A2    // Can use A2 as analog input

// PIR Motion Sensor
#define PIR_PIN          4     // Digital pin for PIR

// Light Sensor (LDR)
#define LDR_PIN          A3    // Can use A3 as analog input

// Slotted Encoder (for distance tracking)
#define ENCODER_PIN      2     // Must use D2 or D3 for interrupt

// LED Indicator
#define LED_PIN          13    // Built-in LED or external

// LCD uses A4 (SDA) and A5 (SCL) - I2C pins (fixed)

// ============================================
// OBJECT INITIALIZATION
// ============================================
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD address (try 0x3F if 0x27 doesn't work)
DHT dht(DHT_PIN, DHT_TYPE);          // DHT11 sensor

// ============================================
// GLOBAL VARIABLES - SENSOR DATA
// ============================================

// Ultrasonic Sensor Variables
float g_distanceCm = 0.0;           // Distance in cm
bool g_obstacleDetected = false;    // Obstacle within threshold

// Moisture Sensor Variables
int g_moistureRaw = 0;              // Raw ADC value (0-1023)
float g_moisturePercent = 0.0;      // Moisture percentage
bool g_needsWatering = false;       // Low moisture flag

// DHT11 Variables
float g_temperature = 0.0;          // Temperature in Celsius
float g_humidity = 0.0;             // Humidity percentage
bool g_dhtReadSuccess = false;      // DHT reading status

// Light Sensor Variables
int g_lightRaw = 0;                 // Raw ADC value
float g_lightPercent = 0.0;         // Light intensity percentage
bool g_sufficientLight = false;     // Adequate light flag

// PIR Motion Sensor Variables
bool g_motionDetected = false;      // Motion detection flag
unsigned long g_lastMotionTime = 0; // Last motion timestamp

// IR Line Tracking Variables (2 sensors)
// For digital IR: LOW(0) = black line, HIGH(1) = white surface
bool g_irLeft = false;              // Left IR sensor (false=on line)
bool g_irRight = false;             // Right IR sensor (false=on line)
String g_lineDirection = "FORWARD"; // Direction: FORWARD, LEFT, RIGHT, STOP
bool g_onLine = false;              // Robot on line flag

// Encoder Variables (Distance Tracking)
volatile unsigned long g_encoderPulses = 0;  // Encoder pulse count (volatile for ISR)
float g_distanceTravelled = 0.0;    // Distance in meters
const float PULSES_PER_METER = 100.0; // Calibrate this value!

// Motor Control Variables
int g_baseSpeed = 150;              // Base speed for movement (0-255)
bool g_motorsRunning = false;       // Motor status
bool g_lineFollowMode = false;      // Line following enabled

// System Timing Variables
unsigned long g_lastSensorUpdate = 0;   // Last sensor read time
unsigned long g_lastIRUpdate = 0;       // Last IR read time
const unsigned long SENSOR_INTERVAL = 2000;  // Read sensors every 2s
const unsigned long IR_INTERVAL = 50;        // Read IR every 50ms (fast)

// ============================================
// FUNCTION PROTOTYPES
// ============================================
int getDistance(int echoPin, int trigPin);
void readAllSensors();
void readIRSensors();
void processLineFollowing();
void controlMotors(int leftSpeed, int rightSpeed);
void moveForward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();
void updateLCD();
void printDebugInfo();
void encoderISR();

// ============================================
// SETUP FUNCTION
// ============================================
void setup() {
  // Initialize Serial Communication
  Serial.begin(9600);
  Serial.println(F("================================="));
  Serial.println(F("Urban Farming Robot - IED Project 3"));
  Serial.println(F("Mobile Environmental Monitoring System"));
  Serial.println(F("=================================\n"));
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Urban Farm Bot");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
  
  // Initialize DHT11
  dht.begin();
  Serial.println(F("DHT11 Temperature & Humidity sensor initialized"));
  
  // Configure Motor Pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  
  // Configure Ultrasonic Sensor
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  
  // Configure IR Sensors (can use analog pins as digital)
  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);
  
  // Configure Other Sensors
  pinMode(PIR_PIN, INPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // Attach Encoder Interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
  
  // Stop motors initially
  stopMotors();
  
  Serial.println(F("All systems initialized!"));
  Serial.println(F("Compulsory Components:"));
  Serial.println(F("  - Arduino UNO with IED Shield"));
  Serial.println(F("  - 2WD Robot Car Base"));
  Serial.println(F("  - LCD Display (16x2 I2C)"));
  Serial.println(F("  - Ultrasonic Sensor HC-SR04"));
  Serial.println(F("  - 2x IR Sensors (Line Tracking)"));
  Serial.println(F("\nOptional Components:"));
  Serial.println(F("  - DHT11 (Temperature & Humidity)"));
  Serial.println(F("  - Moisture Sensor"));
  Serial.println(F("  - PIR Motion Sensor"));
  Serial.println(F("  - LDR Light Sensor"));
  Serial.println(F("  - Slotted Encoder"));
  Serial.println(F("\n"));
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");
  delay(1000);
  
  // Enable line following mode
  g_lineFollowMode = true;
}

// ============================================
// MAIN LOOP FUNCTION
// ============================================
void loop() {
  unsigned long currentTime = millis();
  
  // Read IR sensors frequently for line following
  if (currentTime - g_lastIRUpdate >= IR_INTERVAL) {
    g_lastIRUpdate = currentTime;
    readIRSensors();
  }
  
  // Read other sensors periodically
  if (currentTime - g_lastSensorUpdate >= SENSOR_INTERVAL) {
    g_lastSensorUpdate = currentTime;
    readAllSensors();
    updateLCD();
    printDebugInfo();
  }
  
  // ============================================
  // ROBOT CONTROL LOGIC
  // ============================================
  
  // Priority 1: Obstacle Detection - Stop immediately
  if (g_obstacleDetected) {
    stopMotors();
    digitalWrite(LED_PIN, HIGH);
    Serial.println(F("⚠ OBSTACLE DETECTED - STOPPING!"));
    return; // Skip other logic
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  
  // Priority 2: Line Following Mode
  if (g_lineFollowMode) {
    processLineFollowing();
  }
  
  // Priority 3: Environmental Alerts
  if (g_needsWatering) {
    // Could add watering system logic here
    Serial.println(F("🌱 Low soil moisture detected!"));
  }
  
  if (g_motionDetected) {
    Serial.println(F("🚶 Motion detected in monitored area!"));
  }
  
  // Add your custom robot behavior here
}

// ============================================
// SENSOR READING FUNCTIONS
// ============================================

/**
 * Read all environmental sensors
 */
void readAllSensors() {
  // Read Ultrasonic Distance
  g_distanceCm = getDistance(ULTRASONIC_ECHO, ULTRASONIC_TRIG);
  g_obstacleDetected = (g_distanceCm > 0 && g_distanceCm < 20.0);
  
  // Read Moisture Sensor
  g_moistureRaw = analogRead(MOISTURE_PIN);
  g_moisturePercent = map(g_moistureRaw, 1023, 300, 0, 100); // Calibrate these values!
  g_moisturePercent = constrain(g_moisturePercent, 0, 100);
  g_needsWatering = (g_moisturePercent < 30.0);
  
  // Read DHT11
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  if (!isnan(temp) && !isnan(hum)) {
    g_temperature = temp;
    g_humidity = hum;
    g_dhtReadSuccess = true;
  } else {
    g_dhtReadSuccess = false;
  }
  
  // Read Light Sensor
  g_lightRaw = analogRead(LDR_PIN);
  g_lightPercent = map(g_lightRaw, 1023, 0, 0, 100); // Calibrate!
  g_lightPercent = constrain(g_lightPercent, 0, 100);
  g_sufficientLight = (g_lightPercent > 40.0);
  
  // Read PIR Motion
  if (digitalRead(PIR_PIN) == HIGH) {
    g_motionDetected = true;
    g_lastMotionTime = millis();
  } else if (millis() - g_lastMotionTime > 5000) {
    g_motionDetected = false;
  }
  
  // Calculate distance travelled
  g_distanceTravelled = g_encoderPulses / PULSES_PER_METER;
}

/**
 * Get distance from ultrasonic sensor (from Project3_9 example)
 * @param echoPin: Echo pin number
 * @param trigPin: Trigger pin number
 * @return: Distance in centimeters
 */
int getDistance(int echoPin, int trigPin) {
  long duration;
  int distance;
  
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read echo pulse
  duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  
  // Calculate distance
  distance = duration * 0.034 / 2;
  
  // Handle timeout
  if (duration == 0) {
    distance = 999; // Out of range
  }
  
  return distance;
}

/**
 * Read IR sensors for line tracking (from Project3_10)
 * IR Logic: LOW(0) = black line, HIGH(1) = white surface
 */
void readIRSensors() {
  // Read both IR sensors (using analog pins as digital)
  g_irLeft = digitalRead(IR_SENSOR_1);   // false(0) = on line, true(1) = off line
  g_irRight = digitalRead(IR_SENSOR_2);  // false(0) = on line, true(1) = off line
  
  // Determine direction based on sensor readings
  if (!g_irLeft && !g_irRight) {
    // Both on line - go straight
    g_lineDirection = "FORWARD";
    g_onLine = true;
  }
  else if (!g_irLeft && g_irRight) {
    // Left on line, right off - turn left
    g_lineDirection = "LEFT";
    g_onLine = true;
  }
  else if (g_irLeft && !g_irRight) {
    // Right on line, left off - turn right
    g_lineDirection = "RIGHT";
    g_onLine = true;
  }
  else {
    // Both off line - line lost
    g_lineDirection = "STOP";
    g_onLine = false;
  }
}

/**
 * Process line following based on IR readings
 */
void processLineFollowing() {
  if (!g_onLine) {
    stopMotors();
    Serial.println(F("⚠ Line lost - stopping"));
    return;
  }
  
  if (g_lineDirection == "FORWARD") {
    moveForward(g_baseSpeed);
  }
  else if (g_lineDirection == "LEFT") {
    turnLeft(g_baseSpeed);
  }
  else if (g_lineDirection == "RIGHT") {
    turnRight(g_baseSpeed);
  }
  else {
    stopMotors();
  }
}

// ============================================
// MOTOR CONTROL FUNCTIONS
// Based on MX1508 Driver (Project3_8)
// ============================================

/**
 * Control both motors individually
 * @param leftSpeed: -255 to 255 (negative = backward)
 * @param rightSpeed: -255 to 255 (negative = backward)
 */
void controlMotors(int leftSpeed, int rightSpeed) {
  // Left Motor (Motor A) Control
  if (leftSpeed > 0) {
    // Forward
    analogWrite(MOTOR_A_IN1, leftSpeed);
    digitalWrite(MOTOR_A_IN2, LOW);
  } else if (leftSpeed < 0) {
    // Backward
    digitalWrite(MOTOR_A_IN1, LOW);
    analogWrite(MOTOR_A_IN2, abs(leftSpeed));
  } else {
    // Stop
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
  }
  
  // Right Motor (Motor B) Control
  if (rightSpeed > 0) {
    // Forward
    analogWrite(MOTOR_B_IN3, rightSpeed);
    digitalWrite(MOTOR_B_IN4, LOW);
  } else if (rightSpeed < 0) {
    // Backward
    digitalWrite(MOTOR_B_IN3, LOW);
    analogWrite(MOTOR_B_IN4, abs(rightSpeed));
  } else {
    // Stop
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, LOW);
  }
  
  g_motorsRunning = (leftSpeed != 0 || rightSpeed != 0);
}

/**
 * Move forward at specified speed
 */
void moveForward(int speed) {
  controlMotors(speed, speed);
}

/**
 * Turn left (reduce left motor speed)
 */
void turnLeft(int speed) {
  int turnSpeed = speed * 0.4; // Left at 40% for smooth turn
  controlMotors(turnSpeed, speed);
}

/**
 * Turn right (reduce right motor speed)
 */
void turnRight(int speed) {
  int turnSpeed = speed * 0.4; // Right at 40% for smooth turn
  controlMotors(speed, turnSpeed);
}

/**
 * Stop all motors
 */
void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, LOW);
  g_motorsRunning = false;
}

// ============================================
// LCD DISPLAY FUNCTION
// ============================================
void updateLCD() {
  lcd.clear();
  
  // Display priority information
  if (g_obstacleDetected) {
    lcd.setCursor(0, 0);
    lcd.print("OBSTACLE AHEAD!");
    lcd.setCursor(0, 1);
    lcd.print("Dist: ");
    lcd.print((int)g_distanceCm);
    lcd.print("cm");
  }
  else if (!g_onLine && g_lineFollowMode) {
    lcd.setCursor(0, 0);
    lcd.print("LINE LOST!");
    lcd.setCursor(0, 1);
    lcd.print("L:");
    lcd.print(g_irLeft);
    lcd.print(" R:");
    lcd.print(g_irRight);
  }
  else if (g_needsWatering) {
    lcd.setCursor(0, 0);
    lcd.print("Moisture: ");
    lcd.print((int)g_moisturePercent);
    lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print("NEEDS WATERING!");
  }
  else if (g_dhtReadSuccess) {
    // Normal display
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(g_temperature, 1);
    lcd.print("C H:");
    lcd.print((int)g_humidity);
    lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print("M:");
    lcd.print((int)g_moisturePercent);
    lcd.print("% D:");
    lcd.print(g_distanceTravelled, 1);
    lcd.print("m");
  }
}

// ============================================
// DEBUG OUTPUT FUNCTION
// ============================================
void printDebugInfo() {
  Serial.println(F("\n========== Sensor Data =========="));
  
  // Environmental Data
  Serial.print(F("Temperature: "));
  Serial.print(g_temperature);
  Serial.println(F("°C"));
  
  Serial.print(F("Humidity: "));
  Serial.print(g_humidity);
  Serial.println(F("%"));
  
  Serial.print(F("Soil Moisture: "));
  Serial.print(g_moisturePercent);
  Serial.print(F("% (Raw: "));
  Serial.print(g_moistureRaw);
  Serial.println(F(")"));
  
  Serial.print(F("Light Level: "));
  Serial.print(g_lightPercent);
  Serial.print(F("% (Raw: "));
  Serial.print(g_lightRaw);
  Serial.println(F(")"));
  
  // Distance & Obstacle
  Serial.print(F("Ultrasonic Distance: "));
  Serial.print(g_distanceCm);
  Serial.println(F(" cm"));
  
  Serial.print(F("Obstacle Detected: "));
  Serial.println(g_obstacleDetected ? "YES" : "NO");
  
  // Line Tracking
  Serial.print(F("IR Left: "));
  Serial.print(g_irLeft ? "OFF" : "ON");
  Serial.print(F(" | IR Right: "));
  Serial.println(g_irRight ? "OFF" : "ON");
  
  Serial.print(F("Line Direction: "));
  Serial.println(g_lineDirection);
  
  // Motion & Distance
  Serial.print(F("Motion Detected: "));
  Serial.println(g_motionDetected ? "YES" : "NO");
  
  Serial.print(F("Distance Travelled: "));
  Serial.print(g_distanceTravelled);
  Serial.print(F(" m ("));
  Serial.print(g_encoderPulses);
  Serial.println(F(" pulses)"));
  
  Serial.println(F("=================================\n"));
}

// ============================================
// INTERRUPT SERVICE ROUTINE
// ============================================
void encoderISR() {
  g_encoderPulses++;
}