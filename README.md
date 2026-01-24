# Hexapod Robot – Embedded Systems Project

This repository contains the source code developed for a **six-legged (hexapod) robot**, created in the context of an academic project in **Embedded Computing Systems**.

The project focuses on the control of multiple servomotors, implementation of a finite state machine, and communication with the user through Serial and Wi-Fi interfaces.

---

## 📌 Project Overview

The hexapod robot is capable of performing the following movements:
- Stand up
- Lay down
- Walk forward
- Turn left
- Turn right

The robot uses a **PCA9685 PWM controller** to drive 12 servomotors (two per leg) and includes an **ultrasonic sensor (HC-SR04)** for obstacle detection. A **DC-DC Buck converter** is used to supply the correct voltage to the PCA9685 and servos.

The robot behavior is managed using a **Finite State Machine (FSM)**, allowing clear separation between different operational modes and user commands.

---

## 🧠 Software Architecture

The software is written in **C/C++ using the Arduino framework** and is organized around the following concepts:

- Finite State Machine (FSM) for behavior control
- Modular movement routines (walk, turn, stand, lay)
- I2C communication with the PCA9685
- GPIO-based communication with the HC-SR04 ultrasonic sensor
- User control via Serial Monitor and Wi-Fi (TCP server)

---

## 📁 Important Files

- **`apresentacao.cpp`**  
  👉 **Final and complete version of the project code**  
  This is the version used for the project presentation and evaluation.

Other files (if present) may include earlier versions or test code.

---
