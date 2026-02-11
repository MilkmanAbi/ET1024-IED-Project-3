/*
 * Urban Farming Robot - WebServer
 *
 * WiFi Access Point + WebSocket bridge to Arduino Uno.
 * Serves a responsive web dashboard for robot control and monitoring.
 * Directly drives MG996R continuous rotation servo on GPIO23 (actuator).
 *
 * Hardware: NodeMCU-32S
 *   Serial2 RX (GPIO19) <- voltage divider <- Arduino TX (A3)
 *   Serial2 TX (GPIO21) -> Arduino RX (A2)
 *   Servo signal (GPIO23) -> MG996R signal wire
 *
 * Wiring:   brown wire = GPIO19 (RX)
 *           black wire = GPIO21 (TX)
 *
 * Libraries: WiFi.h (built-in), ESPAsyncWebServer, AsyncTCP
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "web_content.h"


// WiFi AP credentials
const char* AP_SSID = "UrbanFarmBot";
const char* AP_PASS = "farm1234";

// Serial2 pins for Arduino comms (NodeMCU-32S)
#define ARD_RX 19  // RX (GPIO19, brown) <- Arduino TX (A3)
#define ARD_TX 21  // TX (GPIO21, black) -> Arduino RX (A2)
#define ARD_BAUD 9600

// Servo on GPIO23 (MG996R 360° continuous rotation — tuned values)
#define SERVO_PIN          23
#define SERVO_FREQ         50     // Standard servo 50Hz
#define SERVO_RES          16     // 16-bit resolution
// Pulse widths (microseconds) — matches Arduino Servo.write() 544-2400us mapping
#define SERVO_STOP_US    1472     // 90° = stop
#define SERVO_UP_US      1369     // 80° = anticlockwise (first spin = actuator down)
#define SERVO_DOWN_US    1575     // 100° = clockwise (actuator up)
#define ACT_PUSH_DOWN_MS 1000     // Duration for push-down
#define ACT_PULL_UP_MS   1300     // Duration for pull-up (gravity comp)

// Web server + WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Serial buffer from Arduino
char ardBuf[128];
uint8_t ardIdx = 0;

// Client tracking
bool clientConnected = false;

// Periodic polling
unsigned long lastPoll = 0;
#define POLL_INTERVAL 3000  // Request telemetry every 3s

// Heartbeat to Arduino (keeps watchdog happy, prevents brownout-orphaned motors)
unsigned long lastHeartbeat = 0;
#define HEARTBEAT_INTERVAL 1000  // Send 'H' every 1s

// Actuator state machine (runs)
enum ActState { ACT_IDLE, ACT_MOVING_DOWN, ACT_MOVING_UP };
ActState actState = ACT_IDLE;
bool actuatorIsDown = false;
unsigned long actStartTime = 0;

// ============================================================
// Servo helpers (LEDC hardware PWM — no library needed)
// ============================================================

void servoWriteUs(int pulseUs) {
  // 50Hz = 20000us period. 16-bit = 65536 counts.
  uint32_t duty = (uint32_t)pulseUs * 65536 / 20000;
  ledcWrite(SERVO_PIN, duty);
}

void servoStop() {
  ledcWrite(SERVO_PIN, 0);  // No pulses = servo idle
}

// ============================================================
// Actuator control
// ============================================================

void startActuatorDown() {
  if (actuatorIsDown || actState != ACT_IDLE) return;
  servoWriteUs(SERVO_UP_US);  // Anticlockwise (first spin) → actuator down
  actStartTime = millis();
  actState = ACT_MOVING_DOWN;
  // Notify web clients
  if (clientConnected) ws.textAll("A:MOVING");
  Serial.println("Actuator: pushing down");
}

void startActuatorUp() {
  if (!actuatorIsDown || actState != ACT_IDLE) return;
  servoWriteUs(SERVO_DOWN_US);  // Clockwise → actuator up
  actStartTime = millis();
  actState = ACT_MOVING_UP;
  if (clientConnected) ws.textAll("A:MOVING");
  Serial.println("Actuator: pulling up");
}

void updateActuator(unsigned long now) {
  if (actState == ACT_IDLE) return;

  if (actState == ACT_MOVING_DOWN && (now - actStartTime >= ACT_PUSH_DOWN_MS)) {
    servoStop();
    actuatorIsDown = true;
    actState = ACT_IDLE;
    if (clientConnected) ws.textAll("A:DOWN");
    Serial.println("Actuator: DOWN complete");
  }

  if (actState == ACT_MOVING_UP && (now - actStartTime >= ACT_PULL_UP_MS)) {
    servoStop();
    actuatorIsDown = false;
    actState = ACT_IDLE;
    if (clientConnected) ws.textAll("A:UP");
    Serial.println("Actuator: UP complete");
  }
}

// ============================================================
// WebSocket Event Handler
// ============================================================

void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WS client #%u connected\n", client->id());
      clientConnected = true;
      // Request current state from Arduino
      Serial2.println("P");
      // Send actuator state (owns this now)
      delay(10);
      client->text(actuatorIsDown ? "A:DOWN" : "A:UP");
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("WS client #%u disconnected\n", client->id());
      clientConnected = (ws.count() > 0);
      if (!clientConnected) {
        // No one controlling — stop motors for safety
        Serial2.println("S");
        Serial.println("All clients gone — sent stop to Arduino");
      }
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        // Null-terminate incoming data
        char cmd[32];
        size_t copyLen = (len < sizeof(cmd) - 1) ? len : sizeof(cmd) - 1;
        memcpy(cmd, data, copyLen);
        cmd[copyLen] = '\0';

        // Actuator commands — handle locally on
        if (cmd[0] == 'D' && cmd[1] == ':') {
          Serial.printf("WS -> actuator: %s\n", cmd);
          if (cmd[2] == '1') startActuatorDown();
          else if (cmd[2] == '0') startActuatorUp();
          break;
        }

        // Everything else → forward to Arduino
        Serial.printf("WS -> Arduino: %s\n", cmd);
        Serial2.println(cmd);
      }
      break;
    }

    default:
      break;
  }
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println("Urban Farm Bot - Starting...");

  // Arduino serial link (NodeMCU-32S: GPIO19=RX, GPIO21=TX)
  Serial2.begin(ARD_BAUD, SERIAL_8N1, ARD_RX, ARD_TX);

  // Servo on GPIO23 — LEDC hardware PWM
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
  servoStop();  // Ensure stopped on boot

  // WiFi Access Point
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Serve web dashboard
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  server.begin();
  Serial.println("Web server started on port 80");

  // Wait for Arduino to boot, then kill any orphaned motors (brownout recovery)
  delay(2000);
  Serial2.println("S");   // Stop motors first (in case we rebooted from brownout)
  delay(50);
  Serial2.println("P");   // Then request current state
  Serial.println("Boot: sent stop + ping to Arduino");
}

// ============================================================
// Main Loop
// ============================================================

void loop() {
  unsigned long now = millis();

  // Actuator state machine (servo on GPIO23)
  updateActuator(now);

  // Read data from Arduino and broadcast to WebSocket clients
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n' || c == '\r') {
      if (ardIdx > 0) {
        ardBuf[ardIdx] = '\0';
        Serial.printf("Arduino -> WS: %s\n", ardBuf);

        // Broadcast to all connected WebSocket clients
        if (clientConnected) {
          ws.textAll(ardBuf);
        }
        ardIdx = 0;
      }
    } else if (ardIdx < sizeof(ardBuf) - 1) {
      ardBuf[ardIdx++] = c;
    } else {
      // Buffer overflow - reset
      ardIdx = 0;
    }
  }

  // Heartbeat to Arduino — keeps watchdog happy so motors aren't killed
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = now;
    Serial2.println("H");
  }

  // Periodic telemetry poll - keeps dashboard data fresh
  if (clientConnected && (now - lastPoll >= POLL_INTERVAL)) {
    lastPoll = now;
    Serial2.println("P");
  }

  // Periodic WebSocket cleanup
  static unsigned long lastCleanup = 0;
  if (now - lastCleanup > 1000) {
    lastCleanup = now;
    ws.cleanupClients();
  }
}
