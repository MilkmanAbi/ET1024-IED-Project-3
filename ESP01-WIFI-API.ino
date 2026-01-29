#include <SoftwareSerial.h>

// ============================================
// PIN CONFIGURATION
// ============================================
#define RX 12  // Arduino D12 <- ESP8266 TX
#define TX 13  // Arduino D13 -> Voltage Divider -> ESP8266 RX

// ============================================
// WIFI CREDENTIALS
// ============================================
String WIFI_SSID = "Abi-Oppo-A18";    
String WIFI_PASS = "ilovecutemen";  // Leave empty for open networks

// ============================================
// SETUP
// ============================================
SoftwareSerial esp8266(RX, TX); 

void setup() {
  Serial.begin(9600);
  esp8266.begin(9600);
  
  Serial.println("\n==============================");
  Serial.println("ESP8266 WiFi Setup");
  Serial.println("==============================\n");
  
  // Initialize ESP8266
  if(!initializeESP()) {
    Serial.println("ERROR: ESP8266 initialization failed!");
    return;
  }
  
  // Connect to WiFi
  if(!connectWiFi()) {
    Serial.println("ERROR: WiFi connection failed!");
    return;
  }
  
  Serial.println("\n*** SETUP COMPLETE ***");
  Serial.println("You can now use the ESP8266!");
}

void loop() {
  // Your main code here
  
  // Optional: Check WiFi status every 30 seconds
  static unsigned long lastCheck = 0;
  if(millis() - lastCheck > 30000) {
    checkWiFiStatus();
    lastCheck = millis();
  }
}

// ============================================
// HELPER FUNCTIONS
// ============================================

bool initializeESP() {
  Serial.println("Initializing ESP8266...");
  
  // Test communication
  if(!sendCommand("AT", 2, "OK")) {
    Serial.println("✗ ESP8266 not responding");
    return false;
  }
  Serial.println("✓ ESP8266 found");
  
  // Reset module
  Serial.println("Resetting module...");
  sendCommand("AT+RST", 5, "ready");
  delay(2000);
  
  // Disable echo
  sendCommand("ATE0", 2, "OK");
  
  // Set Station mode
  if(!sendCommand("AT+CWMODE=1", 3, "OK")) {
    Serial.println("✗ Failed to set station mode");
    return false;
  }
  Serial.println("✓ Station mode set");
  
  return true;
}

bool connectWiFi() {
  Serial.print("Connecting to: ");
  Serial.println(WIFI_SSID);
  
  String cmd = "AT+CWJAP=\"" + WIFI_SSID + "\",\"" + WIFI_PASS + "\"";
  
  if(!sendCommand(cmd, 20, "OK")) {
    Serial.println("✗ Connection failed");
    return false;
  }
  
  Serial.println("✓ Connected!");
  
  // Get IP address
  Serial.println("\nIP Address:");
  sendCommand("AT+CIFSR", 3, "OK");
  
  return true;
}

void checkWiFiStatus() {
  Serial.println("\n--- WiFi Status ---");
  sendCommand("AT+CWJAP?", 3, "OK");
}

bool sendCommand(String command, int timeoutSeconds, String expectedReply) {
  // Clear buffer
  while(esp8266.available()) esp8266.read();
  
  esp8266.println(command);
  
  String response = "";
  unsigned long startTime = millis();
  
  while (millis() - startTime < (timeoutSeconds * 1000)) {
    while(esp8266.available()) {
      char c = esp8266.read();
      response += c;
      Serial.write(c);  // Show response
    }
    
    if(response.indexOf(expectedReply) != -1) {
      return true;
    }
  }
  
  return false;
}