// ESP8266 - PURE WiFi CHIP (No brain, just radio)
// Arduino is the boss, ESP8266 just does what it's told

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

String ssid = "";
String password = "";
bool configured = false;

WiFiClient client;
HTTPClient http;

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("READY");
}

void loop() {
  // Just listen and obey
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      executeCommand(cmd);
    }
  }
  
  delay(10);
}

void executeCommand(String cmd) {
  
  // CONNECT:SSID:PASSWORD
  if (cmd.startsWith("CONNECT:")) {
    int first = cmd.indexOf(':');
    int second = cmd.indexOf(':', first + 1);
    
    if (second > 0) {
      ssid = cmd.substring(first + 1, second);
      password = cmd.substring(second + 1);
      
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid.c_str(), password.c_str());
      
      Serial.println("CONNECTING");
      
      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        attempts++;
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        configured = true;
        Serial.print("CONNECTED:");
        Serial.println(WiFi.localIP());
      } else {
        Serial.println("FAILED");
      }
    }
  }
  
  // STATUS
  else if (cmd == "STATUS") {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("CONNECTED:");
      Serial.print(WiFi.localIP());
      Serial.print(":");
      Serial.println(WiFi.RSSI());
    } else {
      Serial.println("DISCONNECTED");
    }
  }
  
  // GET:url
  else if (cmd.startsWith("GET:")) {
    String url = cmd.substring(4);
    httpGet(url);
  }
  
  // POST:url:contentType:body
  else if (cmd.startsWith("POST:")) {
    int first = cmd.indexOf(':', 5);
    int second = cmd.indexOf(':', first + 1);
    
    if (second > 0) {
      String url = cmd.substring(5, first);
      String contentType = cmd.substring(first + 1, second);
      String body = cmd.substring(second + 1);
      httpPost(url, contentType, body);
    }
  }
  
  // TCP:host:port:data  (raw TCP)
  else if (cmd.startsWith("TCP:")) {
    int first = cmd.indexOf(':', 4);
    int second = cmd.indexOf(':', first + 1);
    
    if (second > 0) {
      String host = cmd.substring(4, first);
      int port = cmd.substring(first + 1, second).toInt();
      String data = cmd.substring(second + 1);
      tcpSend(host, port, data);
    }
  }
  
  // UDP:host:port:data
  else if (cmd.startsWith("UDP:")) {
    int first = cmd.indexOf(':', 4);
    int second = cmd.indexOf(':', first + 1);
    
    if (second > 0) {
      String host = cmd.substring(4, first);
      int port = cmd.substring(first + 1, second).toInt();
      String data = cmd.substring(second + 1);
      udpSend(host, port, data);
    }
  }
  
  // PING:host
  else if (cmd.startsWith("PING:")) {
    String host = cmd.substring(5);
    pingHost(host);
  }
  
  // IP
  else if (cmd == "IP") {
    Serial.println(WiFi.localIP());
  }
  
  // RSSI
  else if (cmd == "RSSI") {
    Serial.println(WiFi.RSSI());
  }
  
  // DISCONNECT
  else if (cmd == "DISCONNECT") {
    WiFi.disconnect();
    Serial.println("DISCONNECTED");
  }
  
  // PING (test serial)
  else if (cmd == "PING") {
    Serial.println("PONG");
  }
  
  // Unknown
  else {
    Serial.print("UNKNOWN:");
    Serial.println(cmd);
  }
}

void httpGet(String url) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERROR:NOT_CONNECTED");
    return;
  }
  
  http.begin(client, url);
  int code = http.GET();
  
  if (code > 0) {
    Serial.print("HTTP:");
    Serial.println(code);
    
    if (code == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.print("DATA:");
      Serial.println(payload);
    }
  } else {
    Serial.print("ERROR:");
    Serial.println(http.errorToString(code));
  }
  
  http.end();
}

void httpPost(String url, String contentType, String body) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERROR:NOT_CONNECTED");
    return;
  }
  
  http.begin(client, url);
  http.addHeader("Content-Type", contentType);
  
  int code = http.POST(body);
  
  if (code > 0) {
    Serial.print("HTTP:");
    Serial.println(code);
    
    if (code == HTTP_CODE_OK || code == HTTP_CODE_CREATED) {
      String payload = http.getString();
      Serial.print("DATA:");
      Serial.println(payload);
    }
  } else {
    Serial.print("ERROR:");
    Serial.println(http.errorToString(code));
  }
  
  http.end();
}

void tcpSend(String host, int port, String data) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERROR:NOT_CONNECTED");
    return;
  }
  
  WiFiClient tcpClient;
  
  if (tcpClient.connect(host.c_str(), port)) {
    tcpClient.print(data);
    
    Serial.println("TCP:SENT");
    
    // Wait for response
    unsigned long timeout = millis();
    while (tcpClient.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println("TCP:TIMEOUT");
        tcpClient.stop();
        return;
      }
    }
    
    // Read response
    while (tcpClient.available()) {
      String line = tcpClient.readStringUntil('\n');
      Serial.print("TCP:");
      Serial.println(line);
    }
    
    tcpClient.stop();
  } else {
    Serial.println("ERROR:TCP_CONNECT_FAILED");
  }
}

void udpSend(String host, int port, String data) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERROR:NOT_CONNECTED");
    return;
  }
  
  WiFiUDP udp;
  udp.begin(port);
  
  IPAddress ip;
  if (WiFi.hostByName(host.c_str(), ip)) {
    udp.beginPacket(ip, port);
    udp.print(data);
    udp.endPacket();
    
    Serial.println("UDP:SENT");
  } else {
    Serial.println("ERROR:DNS_FAILED");
  }
  
  udp.stop();
}

void pingHost(String host) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERROR:NOT_CONNECTED");
    return;
  }
  
  IPAddress ip;
  if (WiFi.hostByName(host.c_str(), ip)) {
    Serial.print("PING:");
    Serial.println(ip);
  } else {
    Serial.println("ERROR:DNS_FAILED");
  }
}
