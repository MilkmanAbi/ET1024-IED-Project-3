# ET1024-IED-Project-3

Urban Farming Robot — a mobile environmental monitoring system built for the SP IED (Innovation by Design) course, Semester 2. The robot follows a line track while collecting climate data (temperature, humidity, soil moisture, light) and relaying it for smart-farming decisions.

## Components

| Component | Purpose |
|---|---|
| Arduino UNO + IED Shield | Main controller |
| MX1508 Motor Driver + 2WD Base | Movement |
| HC-SR04 Ultrasonic Sensor | Obstacle detection |
| 2x IR Sensors | Line tracking |
| DHT11 / DHT22 | Temperature & humidity |
| Soil Moisture Sensor | Soil moisture level |
| LDR | Light intensity |
| PIR Sensor | Motion detection |
| Slotted Encoder | Distance tracking |
| ESP-01 (ESP8266) | WiFi data relay |
| 16x2 I2C LCD | On-board display |

## Folder Structure

```
ET1024-IED-Project-3/
├── Urban_robot_arduino/          Main integrated sketch (upload this)
│   └── Urban_robot_arduino.ino
├── Modular-Dev-Components/       Individual component test sketches
│   ├── DHT22-Core/               DHT22 temperature & humidity with LCD
│   ├── ESP01-WIFI-API/           ESP8266 WiFi connection via AT commands
│   ├── Line-Follower/            2-sensor IR line following with recovery
│   ├── Servo-Ultrasonic-Core/    Servo-swept ultrasonic scanning
│   ├── Ultrasonic-Sensor-Core/   Basic ultrasonic distance measurement
│   └── WIFI-Minimal/             Minimal ESP8266 WiFi connectivity test
├── Variable Reference Guide - Urban Farming Robot.pdf
├── LICENSE
└── README.md
```

## Required Libraries

Install via Arduino IDE **Sketch > Include Library > Manage Libraries**:

- **LiquidCrystal_I2C** (v1.1.2) — I2C LCD display
- **DHT sensor library** (Adafruit) — DHT11/DHT22 sensors
- **Adafruit Unified Sensor** — required by the DHT library

Built-in (no install needed): `Wire.h`, `SoftwareSerial.h`, `Servo.h`

## WiFi Setup

The ESP-01 sketches use placeholder credentials. Before uploading, update:

```cpp
String WIFI_SSID = "YOUR_SSID";
String WIFI_PASS = "YOUR_PASSWORD";
```

## License

See [LICENSE](LICENSE) for details.
