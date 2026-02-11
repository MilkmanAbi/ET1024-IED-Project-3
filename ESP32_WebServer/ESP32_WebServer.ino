/*
 * Urban Farming Robot - ESP32 WebServer
 *
 * WiFi Access Point + WebSocket bridge to Arduino Uno.
 * Serves a responsive web dashboard for robot control and monitoring.
 *
 * Hardware: NodeMCU-32S
 *   Serial2 RX (GPIO19) <- voltage divider <- Arduino TX (A3)
 *   Serial2 TX (GPIO21) -> Arduino RX (A2)
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
#define ARD_RX 19  // ESP32 RX (GPIO19, brown) <- Arduino TX (A3)
#define ARD_TX 21  // ESP32 TX (GPIO21, black) -> Arduino RX (A2)
#define ARD_BAUD 9600

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
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("WS client #%u disconnected\n", client->id());
      clientConnected = (ws.count() > 0);
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        // Null-terminate incoming data
        char cmd[32];
        size_t copyLen = (len < sizeof(cmd) - 1) ? len : sizeof(cmd) - 1;
        memcpy(cmd, data, copyLen);
        cmd[copyLen] = '\0';

        Serial.printf("WS -> Arduino: %s\n", cmd);

        // Forward command to Arduino
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
  Serial.println("Urban Farm Bot - ESP32 Starting...");

  // Arduino serial link (NodeMCU-32S: GPIO19=RX, GPIO21=TX)
  Serial2.begin(ARD_BAUD, SERIAL_8N1, ARD_RX, ARD_TX);

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

  // Wait for Arduino to boot, then request initial state
  delay(2000);
  Serial2.println("P");
  Serial.println("Sent initial ping to Arduino");
}

// ============================================================
// Main Loop
// ============================================================

void loop() {
  unsigned long now = millis();

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
