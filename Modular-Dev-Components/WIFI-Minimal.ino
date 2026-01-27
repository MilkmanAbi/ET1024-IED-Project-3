#include <SoftwareSerial.h>

#define RX 10
#define TX 11

String AP = "Abi-Oppo-A18";       // CHANGE ME
String PASS = ""; // CHANGE ME

int countTrueCommand;
int countTimeCommand; 
boolean found = false;

SoftwareSerial esp8266(12, 13); 

void setup() {
  Serial.begin(9600);
  esp8266.begin(115200);
  
  Serial.println("ESP8266 WiFi Connection Test");
  Serial.println("=============================");
  
  // Test if ESP8266 is responding
  Serial.println("Testing ESP8266 module...");
  sendCommand("AT", 5, "OK");
  
  // Set to station mode (connect to AP)
  Serial.println("Setting WiFi mode...");
  sendCommand("AT+CWMODE=1", 5, "OK");
  
  // Connect to WiFi
  Serial.println("Connecting to WiFi...");
  sendCommand("AT+CWJAP=\""+ AP +"\",\""+ PASS +"\"", 20, "OK");
  
  // Get connection status
  Serial.println("Checking connection status...");
  sendCommand("AT+CWJAP?", 5, "OK");
  
  // Get IP address
  Serial.println("Getting IP address...");
  sendCommand("AT+CIFSR", 5, "OK");
  
  Serial.println("=============================");
  Serial.println("Setup complete!");
}

void loop() {
  // Check if still connected every 10 seconds
  delay(10000);
  Serial.println("Checking connection...");
  sendCommand("AT+CWJAP?", 5, "OK");
}

void sendCommand(String command, int maxTime, char readReplay[]) {
  Serial.print(countTrueCommand);
  Serial.print(". AT command => ");
  Serial.print(command);
  Serial.print(" ");
  
  found = false;
  countTimeCommand = 0;
  
  while(countTimeCommand < (maxTime * 1)) {
    esp8266.println(command);
    if(esp8266.find(readReplay)) {
      found = true;
      break;
    }
    countTimeCommand++;
  }
  
  if(found == true) {
    Serial.println("OK");
    countTrueCommand++;
    countTimeCommand = 0;
  }
  
  if(found == false) {
    Serial.println("FAIL");
    countTrueCommand = 0;
    countTimeCommand = 0;
  }
  
  // Print ESP8266 response
  delay(1000);
  while(esp8266.available()) {
    Serial.write(esp8266.read());
  }
  Serial.println();
  
  found = false;
}
