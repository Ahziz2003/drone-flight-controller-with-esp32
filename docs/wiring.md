# Hardware Wiring Overview

This document describes the main hardware connections used in the quadcopter project.

---

## Microcontroller
- **ESP32** used as the main flight controller.

---

## IMU Sensor
- IMU (MPU9250) connected to ESP32 via **I2C**.
- SDA → ESP32 SDA(GPIO 19)
- SCL → ESP32 SCL(GPIO 18)
- Powered at 3.3V.

---

## Motors and ESCs
- Four brushless motors controlled by four ESCs).
- ESC signal wires connected to ESP32 PWM/DShot-capable (GPIO 14,25,26,27s.
- ESCs powered directly from the LiPo battery.

---

## Power
- ESP32 powered via a BEC 5V FROM ONE OF ESC.
- Common ground shared between ESP32, IMU, and ESC signal ground.

