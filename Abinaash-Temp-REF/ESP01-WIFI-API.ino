#include <ESP8266WiFi.h>

// ============================================
// WiFi Configuration - CHANGE THESE!
// ============================================
const char* ssid = "YOUR_WIFI_NAME";        // Your WiFi network name
const char* password = "YOUR_WIFI_PASSWORD"; // Your WiFi password

void setup() {
  // Initialize Serial for communication with Arduino
  Serial.begin(9600);
  delay(100);
  
  // Send startup message
  Serial.println("ESP-01 Starting...");
  
  // Initialize built-in LED (GPIO1/TX pin on ESP-01)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED off (inverted on ESP-01)
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  // Wait for connection with LED blinking
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, LOW); // LED on when connected (inverted)
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println();
    Serial.println("WiFi connection failed!");
    Serial.println("Check SSID and password");
  }
  
  Serial.println("Ready for commands");
}

void loop() {
  // Check for commands from Arduino via Serial
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // PING command - test connection
    if (command == "PING") {
      Serial.println("PONG");
    }
    
    // STATUS command - report WiFi status
    else if (command == "STATUS") {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("CONNECTED");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
      } else {
        Serial.println("DISCONNECTED");
      }
    }
    
    // GET command - make HTTP GET request
    else if (command.startsWith("GET:")) {
      String url = command.substring(4);
      url.trim();
      makeHTTPRequest(url);
    }
    
    // POST command - make HTTP POST request
    else if (command.startsWith("POST:")) {
      String url = command.substring(5);
      url.trim();
      Serial.println("POST not implemented yet");
    }
    
    // Unknown command
    else {
      Serial.print("UNKNOWN: ");
      Serial.println(command);
    }
  }
  
  // Auto-reconnect if WiFi drops
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH); // LED off
    Serial.println("WiFi disconnected! Reconnecting...");
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(LED_BUILTIN, LOW); // LED on
      Serial.println("Reconnected!");
      Serial.println(WiFi.localIP());
    }
  }
  
  delay(100);
}

// Function to make HTTP GET request
void makeHTTPRequest(String host) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERROR: Not connected to WiFi");
    return;
  }
  
  WiFiClient client;
  const int httpPort = 80;
  
  Serial.print("Connecting to: ");
  Serial.println(host);
  
  if (!client.connect(host.c_str(), httpPort)) {
    Serial.println("ERROR: Connection failed");
    return;
  }
  
  // Send HTTP request
  client.print(String("GET / HTTP/1.1\r\n") +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");
  
  // Wait for response with timeout
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println("ERROR: Timeout");
      client.stop();
      return;
    }
  }
  
  // Read and send response
  Serial.println("RESPONSE:");
  while (client.available()) {
    String line = client.readStringUntil('\n');
    Serial.println(line);
    
    // Limit output to prevent overflow
    if (line.length() == 0) break; // End of headers
  }
  
  Serial.println("END");
  client.stop();
}
