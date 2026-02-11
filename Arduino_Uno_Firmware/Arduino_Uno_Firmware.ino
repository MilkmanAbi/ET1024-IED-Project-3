/*
 * Urban Farming Robot - Arduino Uno Firmware
 *
 * Hardware: Arduino Uno + IED Shield
 * Sensors: DHT22, HC-SR04 Ultrasonic, IR Line Trackers x2, Capacitive Moisture
 * Actuators: 2x DC Motors (H-bridge), MG996R Servo (linear actuator)
 * Display: 16x2 I2C LCD
 * Comms: SoftwareSerial to ESP32 (9600 baud)
 *
 * A0 is muxed between IR Right (D7=LOW) and Moisture (D7=HIGH).
 * Servo library conflicts with Timer1 PWM on D9/D10 - attach/detach around use.
 */

// ============================================================
// 1. LIBRARIES + PIN DEFINES
// ============================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <DHT.h>

// Motor Driver Pins (IED Shield hardwired - direction swapped to match wiring)
#define MOTOR_A_IN1       6   // Left motor forward
#define MOTOR_A_IN2       9   // Left motor reverse
#define MOTOR_B_IN3       5   // Right motor forward
#define MOTOR_B_IN4       3   // Right motor reverse

// Ultrasonic Sensor (HC-SR04)
#define ULTRASONIC_TRIG  11
#define ULTRASONIC_ECHO  10

// IR Line Tracking
#define IR_LEFT_PIN      A1   // Digital read
#define IR_RIGHT_PIN     A0   // Shared with moisture (muxed via D7)

// DHT22
#define DHT_PIN           2
#define DHT_TYPE       DHT22

// Moisture Sensor (muxed on A0)
#define MOISTURE_ANALOG  A0
#define MOISTURE_POWER    7   // D7: LOW=IR mode, HIGH=moisture mode

// Servo (Linear Actuator)
#define SERVO_PIN         4

// LED Indicator
#define LED_PIN          13

// ESP32 Comms
#define ESP_RX           A2   // Arduino RX <- ESP32 TX
#define ESP_TX           A3   // Arduino TX -> ESP32 RX

// ============================================================
// 2. CONSTANTS
// ============================================================

// Actuator servo angles
#define ACTUATOR_UP_ANGLE    90   // Retracted (neutral)
#define ACTUATOR_DOWN_ANGLE 125   // Extended into soil (90+35)
#define ACTUATOR_TRAVEL_MS  500   // Time budget for 35-degree travel
#define MOISTURE_SETTLE_MS  100   // Settle time after powering moisture sensor

// Motor speeds
#define BASE_SPEED         255
#define TURN_SPEED_FACTOR  0.4f  // Inner wheel at 40% for smooth turns

// Sensor intervals (ms)
#define IR_INTERVAL         50
#define ULTRASONIC_INTERVAL 200
#define TELEMETRY_INTERVAL 3000
#define LCD_INTERVAL        500

// Obstacle threshold (cm)
#define OBSTACLE_DIST_CM    20

// Serial buffer
#define CMD_BUF_SIZE        16

// ============================================================
// 3. GLOBAL VARIABLES + STATE
// ============================================================

// State machine
enum RobotState {
  STATE_IDLE,
  STATE_LINE_SEARCH,
  STATE_LINE_FOLLOW,
  STATE_MANUAL,
  STATE_ACTUATOR_DOWN,
  STATE_ACTUATOR_UP,
  STATE_MOISTURE_READ
};

RobotState currentState = STATE_MANUAL;
RobotState resumeState  = STATE_MANUAL;  // State to return to after actuator ops

// Objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial espSerial(ESP_RX, ESP_TX);
Servo actuatorServo;
DHT dht(DHT_PIN, DHT_TYPE);

// Motor state
bool motorsRunning = false;

// Sensor data
float temperature = 0.0f;
float humidity    = 0.0f;
int   moisturePct = 0;
int   distanceCm  = 999;
bool  irLeft      = false;  // false(0) = on line (white), true(1) = off line (black)
bool  irRight     = false;

// Actuator
bool actuatorIsDown = false;

// Obstacle reporting (avoid spamming)
bool obstacleSent = false;

// Timing (millis-based, non-blocking)
unsigned long lastIRRead        = 0;
unsigned long lastUltrasonic    = 0;
unsigned long lastTelemetry     = 0;
unsigned long lastLCDUpdate     = 0;
unsigned long actuatorStartTime = 0;
unsigned long moistureStartTime = 0;

// Serial command buffer
char cmdBuf[CMD_BUF_SIZE];
uint8_t cmdIdx = 0;

// LCD line buffers
char lcdLine0[17];
char lcdLine1[17];

// Telemetry buffer
char telBuf[48];

// ============================================================
// 4. SETUP
// ============================================================

void setup() {
  // Motor pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  stopMotors();

  // Ultrasonic
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  // IR sensors (analog pins as digital input)
  pinMode(IR_LEFT_PIN, INPUT);
  // IR_RIGHT_PIN (A0) configured dynamically via mux

  // Moisture power gate
  pinMode(MOISTURE_POWER, OUTPUT);
  digitalWrite(MOISTURE_POWER, LOW);  // Default: IR mode on A0

  // LED
  pinMode(LED_PIN, OUTPUT);

  // DHT22
  dht.begin();

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("UrbanFarmBot"));
  lcd.setCursor(0, 1);
  lcd.print(F("Initialising..."));

  // ESP32 serial
  espSerial.begin(9600);

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println(F("Urban Farm Robot ready"));
#endif

  delay(1500);
  lcd.clear();

  // Send initial state so ESP32 knows we're alive
  readDHT();
  distanceCm = readUltrasonic();
  sendTelemetry();
  espSerial.println(F("MODE:MAN"));
  espSerial.println(F("A:UP"));
}

// ============================================================
// 5. MAIN LOOP
// ============================================================

void loop() {
  unsigned long now = millis();

  // Always: read incoming serial commands
  readSerialCommands();

  // State-dependent processing
  switch (currentState) {
    case STATE_LINE_SEARCH:
      // Move forward until an IR sensor detects the black line
      if (now - lastUltrasonic >= ULTRASONIC_INTERVAL) {
        lastUltrasonic = now;
        distanceCm = readUltrasonic();
      }
      if (distanceCm < OBSTACLE_DIST_CM) {
        stopMotors();
        if (!obstacleSent) {
          espSerial.println(F("K:OBSTACLE"));
          obstacleSent = true;
        }
      } else {
        obstacleSent = false;
        controlMotors(BASE_SPEED, BASE_SPEED);  // Drive forward
        if (now - lastIRRead >= IR_INTERVAL) {
          lastIRRead = now;
          readIRSensors();
          if (irLeft || irRight) {
            // Line found - switch to line following
            currentState = STATE_LINE_FOLLOW;
            resumeState = STATE_LINE_FOLLOW;
            espSerial.println(F("MODE:LF"));
          }
        }
      }
      break;

    case STATE_LINE_FOLLOW:
      if (now - lastIRRead >= IR_INTERVAL) {
        lastIRRead = now;
        readIRSensors();
      }
      if (now - lastUltrasonic >= ULTRASONIC_INTERVAL) {
        lastUltrasonic = now;
        distanceCm = readUltrasonic();
      }
      // Obstacle override
      if (distanceCm < OBSTACLE_DIST_CM) {
        stopMotors();
        if (!obstacleSent) {
          espSerial.println(F("K:OBSTACLE"));
          obstacleSent = true;
        }
      } else {
        obstacleSent = false;
        processLineFollowing();
      }
      break;

    case STATE_MANUAL:
      // Motors controlled by ESP32 commands; safety check
      if (now - lastUltrasonic >= ULTRASONIC_INTERVAL) {
        lastUltrasonic = now;
        distanceCm = readUltrasonic();
        if (distanceCm < OBSTACLE_DIST_CM && motorsRunning) {
          stopMotors();
          espSerial.println(F("K:OBSTACLE"));
        }
      }
      break;

    case STATE_ACTUATOR_DOWN:
      if (now - actuatorStartTime >= ACTUATOR_TRAVEL_MS) {
        actuatorIsDown = true;
        actuatorServo.detach();  // Restore Timer1 PWM on D9/D10
        espSerial.println(F("A:DOWN"));
        // Auto-trigger moisture read
        digitalWrite(MOISTURE_POWER, HIGH);
        moistureStartTime = now;
        currentState = STATE_MOISTURE_READ;
      }
      break;

    case STATE_ACTUATOR_UP:
      if (now - actuatorStartTime >= ACTUATOR_TRAVEL_MS) {
        actuatorIsDown = false;
        actuatorServo.detach();  // Restore Timer1 PWM on D9/D10
        espSerial.println(F("A:UP"));
        currentState = resumeState;
      }
      break;

    case STATE_MOISTURE_READ:
      if (now - moistureStartTime >= MOISTURE_SETTLE_MS) {
        int raw = analogRead(MOISTURE_ANALOG);
        moisturePct = map(raw, 1023, 0, 0, 100);  // Capacitive: wet=low value
        if (moisturePct < 0) moisturePct = 0;
        if (moisturePct > 100) moisturePct = 100;
        digitalWrite(MOISTURE_POWER, LOW);  // Back to IR mode
        sendTelemetry();
        // Stay idle (actuator still down, waiting for pull-up command)
        currentState = STATE_IDLE;
      }
      break;

    case STATE_IDLE:
    default:
      break;
  }

  // Periodic: DHT22 + telemetry (every 3s)
  if (now - lastTelemetry >= TELEMETRY_INTERVAL) {
    lastTelemetry = now;
    readDHT();
    if (currentState != STATE_MOISTURE_READ) {
      // Read ultrasonic for telemetry if not recently read
      if (now - lastUltrasonic >= ULTRASONIC_INTERVAL) {
        lastUltrasonic = now;
        distanceCm = readUltrasonic();
      }
      sendTelemetry();
    }
  }

  // Periodic: LCD update (every 500ms)
  if (now - lastLCDUpdate >= LCD_INTERVAL) {
    lastLCDUpdate = now;
    updateLCD();
  }

  // LED: blink in line-follow, solid in manual, off in idle
  updateLED(now);
}

// ============================================================
// 6. SERIAL COMMUNICATION + PROTOCOL PARSING
// ============================================================

void readSerialCommands() {
  while (espSerial.available()) {
    char c = espSerial.read();
    if (c == '\n' || c == '\r') {
      if (cmdIdx > 0) {
        cmdBuf[cmdIdx] = '\0';
        processCommand(cmdBuf);
        cmdIdx = 0;
      }
    } else if (cmdIdx < CMD_BUF_SIZE - 1) {
      cmdBuf[cmdIdx++] = c;
    }
  }
}

void processCommand(const char* cmd) {
#ifdef DEBUG
  Serial.print(F("CMD: "));
  Serial.println(cmd);
#endif

  // Mode commands
  if (cmd[0] == 'M' && cmd[1] == ':') {
    if (cmd[2] == '1') {
      // Manual mode
      stopMotors();
      currentState = STATE_MANUAL;
      resumeState = STATE_MANUAL;
      espSerial.println(F("MODE:MAN"));
    } else if (cmd[2] == '0') {
      // Line follow mode
      currentState = STATE_LINE_FOLLOW;
      resumeState = STATE_LINE_FOLLOW;
      espSerial.println(F("MODE:LF"));
    }
    espSerial.println(F("K:OK"));
    return;
  }

  // Actuator commands
  if (cmd[0] == 'D' && cmd[1] == ':') {
    if (cmd[2] == '1') {
      startActuatorDown();
    } else if (cmd[2] == '0') {
      startActuatorUp();
    }
    espSerial.println(F("K:OK"));
    return;
  }

  // Single-char commands
  switch (cmd[0]) {
    case 'F':
      if (currentState == STATE_MANUAL) moveForward();
      espSerial.println(F("K:OK"));
      break;
    case 'B':
      if (currentState == STATE_MANUAL) moveBackward();
      espSerial.println(F("K:OK"));
      break;
    case 'L':
      if (currentState == STATE_MANUAL) turnLeft();
      espSerial.println(F("K:OK"));
      break;
    case 'R':
      if (currentState == STATE_MANUAL) turnRight();
      espSerial.println(F("K:OK"));
      break;
    case 'S':
      stopMotors();
      espSerial.println(F("K:OK"));
      break;
    case 'G':
      // Start: search for line then follow
      stopMotors();
      currentState = STATE_LINE_SEARCH;
      resumeState = STATE_LINE_SEARCH;
      espSerial.println(F("MODE:SEARCH"));
      espSerial.println(F("K:OK"));
      break;
    case 'X':
      // Stop: go to manual mode
      stopMotors();
      currentState = STATE_MANUAL;
      resumeState = STATE_MANUAL;
      espSerial.println(F("MODE:MAN"));
      espSerial.println(F("K:OK"));
      break;
    case 'P':
      readDHT();
      distanceCm = readUltrasonic();
      sendTelemetry();
      // Also send current mode and actuator state
      if (currentState == STATE_MANUAL || currentState == STATE_IDLE) {
        espSerial.println(F("MODE:MAN"));
      } else if (currentState == STATE_LINE_SEARCH) {
        espSerial.println(F("MODE:SEARCH"));
      } else {
        espSerial.println(F("MODE:LF"));
      }
      delay(5);
      espSerial.println(actuatorIsDown ? F("A:DOWN") : F("A:UP"));
      break;
    default:
#ifdef DEBUG
      Serial.print(F("Unknown cmd: "));
      Serial.println(cmd);
#endif
      break;
  }
}

// ============================================================
// 7. MOTOR CONTROL
// ============================================================

void controlMotors(int leftSpeed, int rightSpeed) {
  // Detach servo to restore Timer1 PWM on pin 9 (left motor)
  if (actuatorServo.attached()) {
    actuatorServo.detach();
  }

  // Left Motor
  if (leftSpeed > 0) {
    analogWrite(MOTOR_A_IN1, leftSpeed);
    digitalWrite(MOTOR_A_IN2, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(MOTOR_A_IN1, LOW);
    analogWrite(MOTOR_A_IN2, -leftSpeed);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
  }

  // Right Motor
  if (rightSpeed > 0) {
    analogWrite(MOTOR_B_IN3, rightSpeed);
    digitalWrite(MOTOR_B_IN4, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(MOTOR_B_IN3, LOW);
    analogWrite(MOTOR_B_IN4, -rightSpeed);
  } else {
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, LOW);
  }

  motorsRunning = (leftSpeed != 0 || rightSpeed != 0);
}

void moveForward() {
  controlMotors(BASE_SPEED, BASE_SPEED);
}

void moveBackward() {
  controlMotors(-BASE_SPEED, -BASE_SPEED);
}

void turnLeft() {
  int inner = (int)(BASE_SPEED * TURN_SPEED_FACTOR);
  controlMotors(inner, BASE_SPEED);
}

void turnRight() {
  int inner = (int)(BASE_SPEED * TURN_SPEED_FACTOR);
  controlMotors(BASE_SPEED, inner);
}

void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, LOW);
  motorsRunning = false;
}

// ============================================================
// 8. SENSOR READING
// ============================================================

int readUltrasonic() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);  // 30ms timeout
  if (duration == 0) return 999;
  return (int)(duration * 0.034f / 2.0f);
}

void readIRSensors() {
  // D7 must be LOW for IR mode on A0
  // IR: 0 = on line (white surface), 1 = off line (black line detected)
  irLeft  = digitalRead(IR_LEFT_PIN);
  irRight = digitalRead(IR_RIGHT_PIN);
}

void readDHT() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) temperature = t;
  if (!isnan(h)) humidity = h;
}

// ============================================================
// 9. LINE FOLLOWING
// ============================================================

void processLineFollowing() {
  // IR: 0 = on line, 1 = off line
  if (!irLeft && !irRight) {
    // Both on line: go forward
    controlMotors(BASE_SPEED, BASE_SPEED);
  }
  else if (irLeft && !irRight) {
    // Left off, right on: turn right
    int inner = (int)(BASE_SPEED * TURN_SPEED_FACTOR);
    controlMotors(BASE_SPEED, inner);
  }
  else if (!irLeft && irRight) {
    // Right off, left on: turn left
    int inner = (int)(BASE_SPEED * TURN_SPEED_FACTOR);
    controlMotors(inner, BASE_SPEED);
  }
  else {
    // Both off line: stop (line lost)
    stopMotors();
  }
}

// ============================================================
// 10. ACTUATOR CONTROL
// ============================================================

void startActuatorDown() {
  if (actuatorIsDown) return;  // Already down

  // Save current state to resume after
  if (currentState == STATE_LINE_SEARCH || currentState == STATE_LINE_FOLLOW || currentState == STATE_MANUAL) {
    resumeState = currentState;
  }

  stopMotors();
  actuatorServo.attach(SERVO_PIN);
  actuatorServo.write(ACTUATOR_DOWN_ANGLE);
  actuatorStartTime = millis();
  currentState = STATE_ACTUATOR_DOWN;
  espSerial.println(F("A:MOVING"));
}

void startActuatorUp() {
  if (!actuatorIsDown) return;  // Already up

  actuatorServo.attach(SERVO_PIN);
  actuatorServo.write(ACTUATOR_UP_ANGLE);
  actuatorStartTime = millis();
  currentState = STATE_ACTUATOR_UP;
  espSerial.println(F("A:MOVING"));
}

// ============================================================
// 11. TELEMETRY
// ============================================================

void sendTelemetry() {
  // Format: T:25.5,H:60.2,M:45,D:15
  char tBuf[7], hBuf[7];
  dtostrf(temperature, 4, 1, tBuf);
  dtostrf(humidity, 4, 1, hBuf);

  snprintf(telBuf, sizeof(telBuf), "T:%s,H:%s,M:%d,D:%d",
           tBuf, hBuf, moisturePct, distanceCm);
  espSerial.println(telBuf);

  // Small delay to let SoftwareSerial finish transmitting
  delay(10);
}

// ============================================================
// 12. LCD DISPLAY
// ============================================================

void updateLCD() {
  // Line 0: State + distance
  const char* stateStr;
  switch (currentState) {
    case STATE_LINE_SEARCH:  stateStr = "SRC"; break;
    case STATE_LINE_FOLLOW:  stateStr = "LF"; break;
    case STATE_MANUAL:       stateStr = "MAN"; break;
    case STATE_ACTUATOR_DOWN:
    case STATE_ACTUATOR_UP:  stateStr = "ACT"; break;
    case STATE_MOISTURE_READ:stateStr = "MST"; break;
    default:                 stateStr = "IDL"; break;
  }
  snprintf(lcdLine0, sizeof(lcdLine0), "%-3s D:%3dcm", stateStr, distanceCm > 999 ? 999 : distanceCm);

  // Line 1: Temp + Humidity or Moisture
  char tBuf[6];
  dtostrf(temperature, 4, 1, tBuf);
  if (actuatorIsDown) {
    snprintf(lcdLine1, sizeof(lcdLine1), "%sC M:%d%%", tBuf, moisturePct);
  } else {
    char hBuf[6];
    dtostrf(humidity, 4, 1, hBuf);
    snprintf(lcdLine1, sizeof(lcdLine1), "%sC H:%s%%", tBuf, hBuf);
  }

  lcd.setCursor(0, 0);
  lcd.print(lcdLine0);
  lcd.setCursor(0, 1);
  lcd.print(lcdLine1);
}

// ============================================================
// 13. LED INDICATOR
// ============================================================

void updateLED(unsigned long now) {
  switch (currentState) {
    case STATE_LINE_SEARCH:
      // Fast blink while searching
      digitalWrite(LED_PIN, (now / 200) % 2);
      break;
    case STATE_LINE_FOLLOW:
      // Blink every 500ms
      digitalWrite(LED_PIN, (now / 500) % 2);
      break;
    case STATE_MANUAL:
      digitalWrite(LED_PIN, HIGH);
      break;
    default:
      digitalWrite(LED_PIN, LOW);
      break;
  }
}
