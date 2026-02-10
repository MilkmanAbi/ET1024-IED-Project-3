#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT22.h>

// LCD (I2C) - Address 0x27, 16 columns, 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

// DHT22 Sensor Setup
#define DHTPIN 4     // Pin D4 as per your schematic
DHT22 dht22(DHTPIN);

void setup() {
  Serial.begin(9600);
  
  lcd.init();
  lcd.backlight();
  
  lcd.setCursor(0, 0);
  lcd.print("DHT22.h D4 Mode");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();
}

void loop() {
  // The DHT22.h library typically uses a read() command
  // This returns a status code (0 is usually success)
  float t = dht22.getTemperature();
  float h = dht22.getHumidity();

  // Validate that the readings are numbers
  if (isnan(t) || isnan(h) || t == -999 || h == -999) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error  ");
    lcd.setCursor(0, 1);
    lcd.print("Retrying...    ");
    Serial.println("Error: Could not read from DHT22");
  } 
  else {
    // Success Display
    lcd.setCursor(0, 0);
    lcd.print("Humidity: ");
    lcd.print(h, 1); // 1 decimal place
    lcd.print("% ");

    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    lcd.print(t, 1); // 1 decimal place
    lcd.print((char)223); 
    lcd.print("C  ");
    
    Serial.print("T: "); Serial.print(t);
    Serial.print(" H: "); Serial.println(h);
  }

  // DHT22 needs a long gap between samples
  delay(2500);
}