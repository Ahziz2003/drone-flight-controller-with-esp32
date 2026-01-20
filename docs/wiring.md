# Hardware Wiring Overview

This document describes the main hardware connections used in the quadcopter project.

---

## Microcontroller
- **ESP32** used as the main flight controller.

---

## IMU Sensor
- IMU connected to ESP32 via **I2C**.
- SDA → ESP32 SDA
- SCL → ESP32 SCL
- Powered at 3.3V.

---

## Motors and ESCs
- Four brushless motors controlled by four ESCs.
- ESC signal wires connected to ESP32 PWM/DShot-capable GPIO pins.
- ESCs powered directly from the LiPo battery.

---

## Power
- ESP32 powered via a regulated 5V or 3.3V supply.
- Common ground shared between ESP32, IMU, and ESC signal ground.

