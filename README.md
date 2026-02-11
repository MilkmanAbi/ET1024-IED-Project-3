# Urban Farming Robot

> Mobile environmental monitoring system built for the SP IED (Innovation by Design) course — Semester 2, Project 3.

A two-wheeled robot that autonomously follows a line track while collecting climate data (temperature, humidity, soil moisture, light) and streaming it to a real-time web dashboard over WiFi.

![Control Panel](control_panel.jpeg)

## Architecture

The system uses a **dual-MCU design**:

- **Arduino Uno** handles real-time sensor reading, motor control, and line-following logic.
- **ESP32** runs a WiFi access point with a WebSocket-based dashboard for remote control and live telemetry.

The two boards communicate over a 9600-baud serial link.

## Hardware

| Component | Role |
|---|---|
| Arduino Uno + IED Shield | Main controller — sensors, motors, line-following |
| ESP32 (NodeMCU-32S) | WiFi AP, web dashboard, servo actuator |
| MX1508 H-Bridge + 2WD Base | Differential drive |
| HC-SR04 Ultrasonic | Obstacle detection |
| 2x IR Sensors | Line tracking |
| DHT22 | Temperature and humidity |
| Soil Moisture Sensor | Soil moisture level (digital) |
| LDR | Light intensity |
| PIR Sensor | Motion detection |
| MG996R Servo (360°) | Actuator mechanism |
| 16x2 I2C LCD | On-board display |

## Folder Structure

```
ET1024-IED-Project-3/
├── Arduino_Uno_Firmware/      Arduino Uno sketch (sensors, motors, serial comms)
├── ESP32_WebServer/           ESP32 sketch (WiFi AP, web dashboard, servo)
├── Urban_robot_arduino/       Legacy integrated sketch
├── control_panel.jpeg         Control panel reference
├── Variable Reference Guide - Urban Farming Robot.pdf
├── LICENSE
└── README.md
```

## Getting Started

### Arduino Uno

Install these libraries via **Sketch > Include Library > Manage Libraries**:

| Library | Notes |
|---|---|
| LiquidCrystal_I2C (v1.1.2) | I2C LCD display |
| DHT sensor library (Adafruit) | DHT22 sensor |
| Adafruit Unified Sensor | Required by DHT library |

Upload `Arduino_Uno_Firmware/Arduino_Uno_Firmware.ino` to the Uno.

### ESP32

Install these libraries:

| Library | Notes |
|---|---|
| ESPAsyncWebServer | Async HTTP + WebSocket server |
| AsyncTCP | Required by ESPAsyncWebServer |

Upload `ESP32_WebServer/ESP32_WebServer.ino` to the ESP32. On boot it creates a WiFi AP:

| Setting | Value |
|---|---|
| SSID | `UrbanFarmBot` |
| Password | `farm1234` |
| Dashboard | `http://192.168.4.1` |

Connect to the AP from any device and open the dashboard to control the robot and view live sensor data.

## License

See [LICENSE](LICENSE) for details.
