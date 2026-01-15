/*
 * Urban Farming Robot - Main Framework
 * Arduino UNO with Multiple Sensors
 * 
 * REQUIRED LIBRARIES TO INSTALL:
 * 1. LiquidCrystal_I2C (Version 1.1.2) - for LCD Display
 *    Install: Sketch -> Include Library -> Manage Libraries -> Search "LiquidCrystal I2C"
 * 
 * 2. DHT sensor library (by Adafruit) - for DHT11 Temperature & Humidity
 *    Install: Sketch -> Include Library -> Manage Libraries -> Search "DHT sensor library"
 * 
 * 3. Adafruit Unified Sensor - Required by DHT library
 *    Install: Sketch -> Include Library -> Manage Libraries -> Search "Adafruit Unified Sensor"
 * 
 * BUILT-IN LIBRARIES (No installation needed):
 * - Wire.h (for I2C communication)
 * - SoftwareSerial.h (for ESP01 communication)
 */

// ============================================
// LIBRARY INCLUDES
// ============================================
#include <Wire.h>                    // I2C communication for LCD
#include <LiquidCrystal_I2C.h>       // LCD Display library
#include <DHT.h>                     // DHT11 sensor library
#include <SoftwareSerial.h>          // For ESP01 WiFi module communication

// ============================================
// PIN DEFINITIONS (Adjust these as needed)
// ============================================

// Motor Driver Pins (AR-MX1508)
#define MOTOR_LEFT_FWD    3    // PWM pin - Left motor forward
#define MOTOR_LEFT_BWD    5    // PWM pin - Left motor backward
#define MOTOR_RIGHT_FWD   6    // PWM pin - Right motor forward
#define MOTOR_RIGHT_BWD   9    // PWM pin - Right motor backward

// Ultrasonic Sensor (HC-SR04)
#define ULTRASONIC_TRIG   7    // Trigger pin
#define ULTRASONIC_ECHO   8    // Echo pin

// DHT11 Temperature & Humidity Sensor
#define DHT_PIN          4     // DHT11 data pin
#define DHT_TYPE         DHT11 // DHT sensor type

// IR Line Tracking Sensors (2 sensors - Digital Output)
// Placed at front bottom for route tracking
#define IR_LEFT_PIN      A2    // Left IR sensor
#define IR_RIGHT_PIN     A3    // Right IR sensor
// IR Logic: LOW (0) = Line detected (black), HIGH (1) = No line (white)

// Slotted Encoder (for distance tracking)
#define ENCODER_PIN      2     // Must be pin 2 or 3 for interrupt

// PIR Motion Sensor
#define PIR_PIN          12    // PIR sensor output pin

// LED Output
#define LED_PIN          13    // LED indicator pin

// Analog Sensors
#define MOISTURE_PIN     A0    // Moisture sensor analog pin
#define LDR_PIN          A1    // Light sensor (LDR) analog pin

// ESP01 WiFi Module (Software Serial)
#define ESP_RX_PIN       10    // Connect to ESP01 TX
#define ESP_TX_PIN       11    // Connect to ESP01 RX

// I2C Pins (Fixed on Arduino UNO - No need to define)
// SDA = A4
// SCL = A5

// ============================================
// OBJECT INITIALIZATION
// ============================================
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD address 0x27 (change to 0x3F if needed)
DHT dht(DHT_PIN, DHT_TYPE);          // DHT11 sensor object
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN); // ESP01 communication

// ============================================
// GLOBAL VARIABLES - SENSOR DATA
// ============================================

// Ultrasonic Sensor Variables
float g_distanceCm = 0.0;           // Distance measured in cm
bool g_obstacleDetected = false;    // Flag for obstacle detection

// Moisture Sensor Variables
int g_moistureRaw = 0;              // Raw ADC value (0-1023)
float g_moisturePercent = 0.0;      // Moisture percentage (0-100%)
bool g_needsWatering = false;       // Flag for watering requirement

// DHT11 Sensor Variables
float g_temperature = 0.0;          // Temperature in Celsius
float g_humidity = 0.0;             // Humidity in percentage
bool g_dhtReadSuccess = false;      // Flag for successful DHT reading

// Light Sensor (LDR) Variables
int g_lightRaw = 0;                 // Raw ADC value (0-1023)
float g_lightPercent = 0.0;         // Light intensity percentage
bool g_sufficientLight = false;     // Flag for adequate light

// PIR Motion Sensor Variables
bool g_motionDetected = false;      // Motion detection flag
unsigned long g_lastMotionTime = 0; // Timestamp of last motion

// IR Line Tracking Sensor Variables (2 sensors)
// Digital: LOW (0) = Line detected (black), HIGH (1) = No line (white)
bool g_irLeft = false;              // Left IR sensor state (false=on line, true=off line)
bool g_irRight = false;             // Right IR sensor state (false=on line, true=off line)
String g_lineDirection = "FORWARD"; // Current direction: "FORWARD", "LEFT", "RIGHT", "STOP"
bool g_onLine = false;              // Flag indicating if robot is on line
bool g_bothOnLine = false;          // Both sensors on line (T-junction or straight)

// Encoder Variables (for distance tracking)
volatile unsigned long g_encoderPulses = 0;  // Total encoder pulses (volatile for interrupt)
float g_distanceTravelled = 0.0;    // Distance travelled in meters
const float PULSES_PER_METER = 100.0; // Calibrate this value for your wheel

// Motor Control Variables
int g_motorSpeed = 0;               // Current motor speed (0-255)
int g_baseSpeed = 150;              // Base speed for line following (adjust as needed)
bool g_motorsRunning = false;       // Motors running state
bool g_lineFollowMode = false;      // Line following mode active

// System Status Variables
unsigned long g_lastUpdateTime = 0;  // Last sensor update timestamp
const unsigned long UPDATE_INTERVAL = 2000; // Update sensors every 2 seconds
unsigned long g_lastIRReadTime = 0;  // Last IR sensor read time
const unsigned long IR_READ_INTERVAL = 50; // Read IR sensors every 50ms (fast response)

// ESP01 Communication Variables
String g_wifiData = "";             // Data to send via WiFi
bool g_wifiConnected = false;       // WiFi connection status

// ============================================
// FUNCTION PROTOTYPES
// ============================================
void readUltrasonicSensor();
void readMoistureSensor();
void readDHT11Sensor();
void readLightSensor();
void readPIRSensor();
void readIRSensors();
void processLineFollowing();
void updateLCD();
void controlMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void moveForward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void sharpTurnLeft(int speed);
void sharpTurnRight(int speed);
void sendDataToESP01();
void encoderISR();

// ============================================
// SETUP FUNCTION
// ============================================
void setup() {
  // Initialize Serial Communication
  Serial.begin(9600);
  Serial.println(F("Urban Farming Robot Initializing..."));
  
  // Initialize ESP01 Serial
  espSerial.begin(9600);
  
  // Initialize LCD Display
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Farm Robot");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
  
  // Initialize DHT Sensor
  dht.begin();
  Serial.println(F("DHT11 initialized"));
  
  // Configure Pin Modes
  // Motor pins
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);
  
  // Ultrasonic sensor pins
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  
  // IR sensor pins (digital input) - 2 sensors
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  
  // Digital input pins
  pinMode(PIR_PIN, INPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  
  // LED output
  pinMode(LED_PIN, OUTPUT);
  
  // Attach interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
  
  // Initial motor stop
  stopMotors();
  
  Serial.println(F("All systems initialized!"));
  Serial.println(F("IR Sensors: 2 (Left & Right) for line tracking"));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  delay(1000);
  
  // Enable line following mode by default
  g_lineFollowMode = true;
}

// ============================================
// MAIN LOOP FUNCTION
// ============================================
void loop() {
  unsigned long currentTime = millis();
  
  // Read IR sensors frequently for responsive line following
  if (currentTime - g_lastIRReadTime >= IR_READ_INTERVAL) {
    g_lastIRReadTime = currentTime;
    readIRSensors();
  }
  
  // Update other sensors at regular intervals
  if (currentTime - g_lastUpdateTime >= UPDATE_INTERVAL) {
    g_lastUpdateTime = currentTime;
    
    // Read all sensors
    readUltrasonicSensor();
    readMoistureSensor();
    readDHT11Sensor();
    readLightSensor();
    readPIRSensor();
    
    // Calculate distance travelled
    g_distanceTravelled = g_encoderPulses / PULSES_PER_METER;
    
    // Update LCD with sensor data
    updateLCD();
    
    // Send data to ESP01
    sendDataToESP01();
    
    // Print debug info to Serial Monitor
    printDebugInfo();
  }
  
  // ============================================
  // ROBOT CONTROL LOGIC
  // ============================================
  
  // Priority 1: Stop if obstacle detected
  if (g_obstacleDetected) {
    stopMotors();
    digitalWrite(LED_PIN, HIGH);  // Turn on LED as warning
    Serial.println(F("Obstacle detected - stopping!"));
    delay(100);
    return; // Skip other logic
  }
  
  // Priority 2: Line following mode
  if (g_lineFollowMode) {
    processLineFollowing();
  }
  
  // Priority 3: Motion detection alert
  if (g_motionDetected) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  
  // Add your custom robot behavior here
  
}

// ============================================
// SENSOR READING FUNCTIONS
// ============================================

/**
 * Read Ultrasonic Sensor (HC-SR04)
 * Updates: g_distanceCm, g_obstacleDetected
 */
void readUltrasonicSensor() {
  // Send trigger pulse
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Read echo pulse duration
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // 30ms timeout
  
  // Calculate distance (speed of sound = 343 m/s)
  g_distanceCm = duration * 0.0343 / 2.0;
  
  // Check for obstacle (less than 20cm)
  g_obstacleDetected = (g_distanceCm > 0 && g_distanceCm < 20.0);
  
  // Handle out of range readings
  if (duration == 0) {
    g_distanceCm = 999.0; // Out of range
  }
}

/**
 * Read Moisture Sensor
 * Updates: g_moistureRaw, g_moisturePercent, g_needsWatering
 */
void readMoistureSensor() {
  g_moistureRaw = analogRead(MOISTURE_PIN);
  
  // Convert to percentage (adjust these values based on calibration)
  // Typical: Dry = 1023, Wet = 300
  g_moisturePercent = map(g_moistureRaw, 1023, 300, 0, 100);
  g_moisturePercent = constrain(g_moisturePercent, 0, 100);
  
  // Check if watering needed (below 30% moisture)
  g_needsWatering = (g_moisturePercent < 30.0);
}

/**
 * Read DHT11 Temperature & Humidity Sensor
 * Updates: g_temperature, g_humidity, g_dhtReadSuccess
 */
void readDHT11Sensor() {
  // Read temperature and humidity
  float temp = dht.readTemperature();  // Celsius
  float hum = dht.readHumidity();
  
  // Check if readings are valid
  if (isnan(temp) || isnan(hum)) {
    Serial.println(F("DHT11 reading failed!"));
    g_dhtReadSuccess = false;
  } else {
    g_temperature = temp;
    g_humidity = hum;
    g_dhtReadSuccess = true;
  }
}

/**
 * Read Light Sensor (LDR)
 * Updates: g_lightRaw, g_lightPercent, g_sufficientLight
 */
void readLightSensor() {
  g_lightRaw = analogRead(LDR_PIN);
  
  // Convert to percentage (adjust based on calibration)
  // Lower value = more light (LDR resistance decreases with light)
  g_lightPercent = map(g_lightRaw, 1023, 0, 0, 100);
  g_lightPercent = constrain(g_lightPercent, 0, 100);
  
  // Check if sufficient light (above 40%)
  g_sufficientLight = (g_lightPercent > 40.0);
}

/**
 * Read PIR Motion Sensor
 * Updates: g_motionDetected, g_lastMotionTime
 */
void readPIRSensor() {
  int pirState = digitalRead(PIR_PIN);
  
  if (pirState == HIGH) {
    g_motionDetected = true;
    g_lastMotionTime = millis();
  } else {
    // Consider motion ended if no detection for 5 seconds
    if (millis() - g_lastMotionTime > 5000) {
      g_motionDetected = false;
    }
  }
}

/**
 * Read IR Line Tracking Sensors (2 sensors)
 * Updates: g_irLeft, g_irRight, g_lineDirection, g_onLine, g_bothOnLine
 * 
 * IR Sensor Logic:
 * - LOW (0) = Black line detected
 * - HIGH (1) = White surface (no line)
 * 
 * 2-Sensor Configuration:
 * [LEFT]  [RIGHT]  -> Robot positioned at front bottom
 * 
 * Logic for 2 sensors:
 * LEFT=0, RIGHT=0  -> Both on line (T-junction or going straight)
 * LEFT=0, RIGHT=1  -> Left on line, Right off -> Turn LEFT
 * LEFT=1, RIGHT=0  -> Left off, Right on line -> Turn RIGHT
 * LEFT=1, RIGHT=1  -> Both off line -> STOP/Search
 */
void readIRSensors() {
  // Read both IR sensors
  g_irLeft = digitalRead(IR_LEFT_PIN);    // false(0)=on line, true(1)=off line
  g_irRight = digitalRead(IR_RIGHT_PIN);  // false(0)=on line, true(1)=off line
  
  // Determine robot action based on sensor readings
  if (!g_irLeft && !g_irRight) {
    // Both sensors on line - go straight or at junction
    g_lineDirection = "FORWARD";
    g_onLine = true;
    g_bothOnLine = true;
  }
  else if (!g_irLeft && g_irRight) {
    // Only left sensor on line - turn left to get back on track
    g_lineDirection = "LEFT";
    g_onLine = true;
    g_bothOnLine = false;
  }
  else if (g_irLeft && !g_irRight) {
    // Only right sensor on line - turn right to get back on track
    g_lineDirection = "RIGHT";
    g_onLine = true;
    g_bothOnLine = false;
  }
  else {
    // Both sensors off line - line lost, stop
    g_lineDirection = "STOP";
    g_onLine = false;
    g_bothOnLine = false;
  }
}

// ============================================
// LINE FOLLOWING CONTROL FUNCTION
// ============================================

/**
 * Process line following logic based on IR sensor readings
 * Controls the robot to follow the line using 2 IR sensors
 */
void processLineFollowing() {
  if (!g_onLine) {
    // Line lost - stop
    stopMotors();
    Serial.println(F("Line lost! Both sensors off line."));
    return;
  }
  
  // Control based on line direction
  if (g_lineDirection == "FORWARD") {
    // Both sensors on line - go straight
    moveForward(g_baseSpeed);
    Serial.println(F("Following line: FORWARD"));
  }
  else if (g_lineDirection == "LEFT") {
    // Left sensor on line, right off - turn left
    turnLeft(g_baseSpeed);
    Serial.println(F("Adjusting: Turn LEFT"));
  }
  else if (g_lineDirection == "RIGHT") {
    // Right sensor on line, left off - turn right
    turnRight(g_baseSpeed);
    Serial.println(F("Adjusting: Turn RIGHT"));
  }
  else {
    // Unknown state - stop
    stopMotors();
  }
}

// ============================================
// MOTOR CONTROL FUNCTIONS
// ============================================

/**
 * Control both motors
 * @param leftSpeed: -255 to 255 (negative = backward)
 * @param rightSpeed: -255 to 255 (negative = backward)
 */
void controlMotors(int leftSpeed, int rightSpeed) {
  // Left motor control
  if (leftSpeed > 0) {
    analogWrite(MOTOR_LEFT_FWD, leftSpeed);
    analogWrite(MOTOR_LEFT_BWD, 0);
  } else if (leftSpeed < 0) {
    analogWrite(MOTOR_LEFT_FWD, 0);
    analogWrite(MOTOR_LEFT_BWD, abs(leftSpeed));
  } else {
    analogWrite(MOTOR_LEFT_FWD, 0);
    analogWrite(MOTOR_LEFT_BWD, 0);
  }
  
  // Right motor control
  if (rightSpeed > 0) {
    analogWrite(MOTOR_RIGHT_FWD, rightSpeed);
    analogWrite(MOTOR_RIGHT_BWD, 0);
  } else if (rightSpeed < 0) {
    analogWrite(MOTOR_RIGHT_FWD, 0);
    analogWrite(MOTOR_RIGHT_BWD, abs(rightSpeed));
  } else {
    analogWrite(MOTOR_RIGHT_FWD, 0);
    analogWrite(MOTOR_RIGHT_BWD, 0);
  }
  
  g_motorsRunning = (leftSpeed != 0 || rightSpeed != 0);
  g_motorSpeed = max(abs(leftSpeed), abs(rightSpeed));
}

/**
 * Stop all motors
 */
void stopMotors() {
  analogWrite(MOTOR_LEFT_FWD, 0);
  analogWrite(MOTOR_LEFT_BWD, 0);
  analogWrite(MOTOR_RIGHT_FWD, 0);
  analogWrite(MOTOR_RIGHT_BWD, 0);
  g_motorsRunning = false;
  g_motorSpeed = 0;
}

/**
 * Move forward at specified speed
 */
void moveForward(int speed) {
  controlMotors(speed, speed);
}

/**
 * Turn left by reducing left motor speed (gentle turn)
 * Good for following curves
 */
void turnLeft(int speed) {
  int turnSpeed = speed * 0.4; // Left motor at 40% speed for smooth turn
  controlMotors(turnSpeed, speed);
}

/**
 * Turn right by reducing right motor speed (gentle turn)
 * Good for following curves
 */
void turnRight(int speed) {
  int turnSpeed = speed * 0.4; // Right motor at 40% speed for smooth turn
  controlMotors(speed, turnSpeed);
}

/**
 * Sharp left turn (for 90-degree turns at intersections)
 * One motor forward, one motor stopped or backward
 */
void sharpTurnLeft(int speed) {
  controlMotors(0, speed);  // Left motor stop, right motor forward
  // Or for spot turn: controlMotors(-speed, speed);
}

/**
 * Sharp right turn (for 90-degree turns at intersections)
 * One motor forward, one motor stopped or backward
 */
void sharpTurnRight(int speed) {
  controlMotors(speed, 0);  // Left motor forward, right motor stop
  // Or for spot turn: controlMotors(speed, -speed);
}

// ============================================
// LCD DISPLAY FUNCTION
// ============================================
void updateLCD() {
  lcd.clear();
  
  // Display different info based on priority
  if (g_obstacleDetected) {
    lcd.setCursor(0, 0);
    lcd.print("OBSTACLE!");
    lcd.setCursor(0, 1);
    lcd.print("Dist:");
    lcd.print(g_distanceCm, 1);
    lcd.print("cm");
  } 
  else if (g_lineFollowMode && !g_onLine) {
    lcd.setCursor(0, 0);
    lcd.print("LINE LOST!");
    lcd.setCursor(0, 1);
    lcd.print("L:");
    lcd.print(g_irLeft);
    lcd.print(" R:");
    lcd.print(g_irRight);
  }
  else if (g_lineFollowMode && g_onLine) {
    lcd.setCursor(0, 0);
    lcd.print("Tracking: ");
    lcd.print(g_lineDirection);
    lcd.setCursor(0, 1);
    lcd.print("L:");
    lcd.print(g_irLeft);
    lcd.print(" R:");
    lcd.print(g_irRight);
    lcd.print(" D:");
    lcd.print(g_distanceTravelled, 1);
  }
  else if (g_needsWatering) {
    lcd.setCursor(0, 0);
    lcd.print("Moisture:");
    lcd.print(g_moisturePercent, 0);
    lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print("Needs Water!");
  } 
  else if (g_dhtReadSuccess) {
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(g_temperature, 1);
    lcd.print("C H:");
    lcd.print(g_humidity, 0);
    lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print("L:");
    lcd.print(g_lightPercent, 0);
    lcd.print("% D:");
    lcd.print(g_distanceTravelled, 2);
    lcd.print("m");
  }
}

// ============================================
// ESP01 COMMUNICATION FUNCTION
// ============================================
void sendDataToESP01() {
  // Create JSON-like data string
  g_wifiData = "{";
  g_wifiData += "\"temp\":";
  g_wifiData += String(g_temperature, 1);
  g_wifiData += ",\"hum\":";
  g_wifiData += String(g_humidity, 1);
  g_wifiData += ",\"moist\":";
  g_wifiData += String(g_moisturePercent, 1);
  g_wifiData += ",\"light\":";
  g_wifiData += String(g_lightPercent, 1);
  g_wifiData += ",\"dist\":";
  g_wifiData += String(g_distanceCm, 1);
  g_wifiData += ",\"motion\":";
  g_wifiData += g_motionDetected ? "1" : "0";
  g_wifiData += ",\"irL\":";
  g_wifiData += g_irLeft ? "1" : "0";
  g_wifiData += ",\"irR\":";
  g_wifiData += g_irRight ? "1" : "0";
  g_wifiData += ",\"line\":\"";
  g_wifiData += g_lineDirection;
  g_wifiData += "\",\"travelled\":";
  g_wifiData += String(g_distanceTravelled, 2);
  g_wifiData += "}";
  
  // Send to ESP01
  espSerial.println(g_wifiData);
}

// ============================================
// INTERRUPT SERVICE ROUTINE
// ============================================
void encoderISR() {
  g_encoderPulses++;
}

// ============================================
// DEBUG FUNCTION
// ============================================
void printDebugInfo() {
  Serial.println(F("\n========== Sensor Readings =========="));
  
  // IR Sensor Status
  Serial.println(F("--- IR Line Tracking ---"));
  Serial.print(F("Left IR: "));
  Serial.print(g_irLeft ? "OFF" : "ON");
  Serial.print(F(" | Right IR: "));
  Serial.println(g_irRight ? "OFF" : "ON");
  Serial.print(F("Direction: "));
  Serial.print(g_lineDirection);
  Serial.print(F(" | On Line: "));
  Serial.println(g_onLine ? "YES" : "NO");
  
  // Environmental Sensors
  Serial.println(F("--- Environmental ---"));
  Serial.print(F("Temperature: "));
  Serial.print(g_temperature);
  Serial.println(F(" C"));
  
  Serial.print(F("Humidity: "));
  Serial.print(g_humidity);
  Serial.println(F(" %"));
  
  Serial.print(F("Moisture: "));
  Serial.print(g_moisturePercent);
  Serial.print(F(" % (Raw: "));
  Serial.print(g_moistureRaw);
  Serial.println(F(")"));
  
  Serial.print(F("Light: "));
  Serial.print(g_lightPercent);
  Serial.print(F(" % (Raw: "));
  Serial.print(g_lightRaw);
  Serial.println(F(")"));
  
  // Distance Sensors
  Serial.println(F("--- Distance ---"));
  Serial.print(F("Ultrasonic: "));
  Serial.print(g_distanceCm);
  Serial.println(F(" cm"));
  
  Serial.print(F("Distance Travelled: "));
  Serial.print(g_distanceTravelled);
  Serial.print(F(" m ("));
  Serial.print(g_encoderPulses);
  Serial.println(F(" pulses)"));
  
  // Motion
  Serial.print(F("Motion Detected: "));
  Serial.println(g_motionDetected ? "YES" : "NO");
  
  Serial.println(F("=====================================\n"));
}