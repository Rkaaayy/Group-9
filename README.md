# EV Automation with AI Planning and Firebase Logging 🚗⚡

This project automates Electric Vehicle (EV) component behavior using sensor inputs, cost-based AI planning (Fast Downward), and real-time status visualization through an OLED display and Firebase.

## 🔧 Features

- **Sensor-based Monitoring**: Reads battery voltage, temperature, RPM, and charging status using Arduino and Raspberry Pi GPIO.
- **AI Planning**: Uses the Fast Downward planner with cost-based planning (`lazy_wastar([ff()], w=1)`) to make decisions.
- **Simulation Modes**: Supports multiple predefined states (`low`, `medium`, `high`, `full`, `high_temp`) and a `cycle` mode to simulate full battery lifecycle.
- **OLED Dashboard**: Displays system status such as RPM, battery %, temperature, charging state, and device states.
- **Firebase Logging**: Real-time push of data to Firebase including sensor values and device states.
- **Safety Constraints**: Prevents motor activation in high temperature conditions.

## 🚀 Quick Start

### 🧱 Hardware Setup

- Raspberry Pi (with GPIO)
- Arduino (sends voltage and battery % over serial)
- L298N Motor Driver (for motor control)
- Sensors:
  - Temperature (e.g., DS18B20)
  - RPM Sensor
  - IR Charging Detection
- OLED Display (128x128 SPI)
