#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD (I2C)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Motor pins (PWM capable pins: 3, 5, 6, 9)
#define LM_IN1 9
#define LM_IN2 6
#define RM_IN1 3
#define RM_IN2 5

// IR sensor pins
#define IR1Pin A0
#define IR2Pin A1

/** * EXPERIMENTAL CALIBRATION */
int leftBaseSpeed = 130;  
int rightBaseSpeed = 110; 
int turnSpeed = 100;      
int recoverSpeed = 100;    // Speed for backing up (increased for better torque)

void setup() {
  pinMode(LM_IN1, OUTPUT);
  pinMode(LM_IN2, OUTPUT);
  pinMode(RM_IN1, OUTPUT);
  pinMode(RM_IN2, OUTPUT);

  pinMode(IR1Pin, INPUT);
  pinMode(IR2Pin, INPUT);

  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  delay(1000);
  lcd.clear();
}

void loop() {
  int IR1 = digitalRead(IR1Pin); // Left Sensor
  int IR2 = digitalRead(IR2Pin); // Right Sensor

  updateLCD(IR1, IR2, "Active");

  if (IR1 == 0 && IR2 == 0) {
    // Both on white: Move Forward
    move(leftBaseSpeed, rightBaseSpeed);
  }
  else if (IR1 == 1 && IR2 == 0) {
    // Left sensor hits black: Pivot Left
    move(0, turnSpeed); 
  }
  else if (IR1 == 0 && IR2 == 1) {
    // Right sensor hits black: Pivot Right
    move(turnSpeed, 0);
  }
  else {
    // BOTH BLACK: Recovery Mechanism
    recover();
  }

  delay(10); 
}

/**
 * Recovery Mechanism: Backs up bit-by-bit until a sensor clears the black line.
 */
void recover() {
  lcd.clear();
  lcd.print("RECOVERING...");
  
  // Step 1: Full Stop first to protect motors
  move(0, 0);
  delay(200);

  // Step 2: Tiny pulses backwards until at least one sensor is on white (0)
  while (digitalRead(IR1Pin) == 1 && digitalRead(IR2Pin) == 1) {
    // Reverse motors
    digitalWrite(LM_IN1, LOW);
    analogWrite(LM_IN2, recoverSpeed);
    digitalWrite(RM_IN1, LOW);
    analogWrite(RM_IN2, recoverSpeed);
    
    delay(100); // Back up for 100ms
    
    // Brief pause to stabilize and check sensors
    digitalWrite(LM_IN2, LOW);
    digitalWrite(RM_IN2, LOW);
    delay(50); 
    
    // Safety exit if it gets stuck in loop (optional)
  }
  
  lcd.clear();
  lcd.print("LINE FOUND");
  delay(200);
}

/**
 * Helper function for forward movement
 */
void move(int leftPwm, int rightPwm) {
  analogWrite(LM_IN1, leftPwm);
  digitalWrite(LM_IN2, LOW);
  analogWrite(RM_IN1, rightPwm);
  digitalWrite(RM_IN2, LOW);
}

void updateLCD(int s1, int s2, String status) {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 300) { 
    lcd.setCursor(0, 0);
    lcd.print("L:"); lcd.print(s1);
    lcd.print(" R:"); lcd.print(s2);
    lcd.print(" Mode: ");
    lcd.setCursor(0, 1);
    lcd.print(status);
    lcd.print("          ");
    lastUpdate = millis();
  }
}
