# 🏁 Autonomous Maze Solving Robot — UoM Maze Solver Challenge 2025
![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![Platform](https://img.shields.io/badge/platform-Arduino-blue)
![Competition](https://img.shields.io/badge/UoM-Maze_Solver_2025-orange)

This repository contains the complete source code, documentation, hardware designs, and media files for our autonomous mobile robot developed for the **Maze Solver Challenge 2025 at the University of Moratuwa**.  
The robot autonomously explores a **16×16 micromouse-style maze**, maps walls, computes an optimal path using **flood-fill**, and performs a fast "speed run" to the finish tile.

---

# 📖 **Abstract**
Maze solving is a world-recognized robotics challenge that originated in the late 1970s. The robot must start from a corner cell, explore the maze, detect walls, map the environment, and reach the goal (a white tile). Once the shortest path is found, the robot performs a high-speed optimized run.

This project implements a fully autonomous maze-solving robot using:
- Arduino Uno (ATmega328P)  
- PID-based wall following   
- Flood-fill path planning  
- A 3D-printed chassis designed in the UoM Department of Electrical Engineering  

---

# 🌀 G16 – MicroMouse / Maze Solver Challenge 2025
### Department of Electrical Engineering – University of Moratuwa

---

## 📌 Project Overview

This repository contains the complete work of **Team G16** for the UoM Intra-Department MicroMouse / Maze Solver Challenge 2025.

The challenge required building an autonomous maze-solving robot capable of navigating a $9 \times 9$ grid using minimal sensing hardware and real-time decision-making. The robot was designed to interpret its surroundings using only **ultrasonic sensors** and an **IR finish detector**, combined with optimized motor control.



> **Key Constraint:** With no encoders and no advanced feedback systems, the project demanded a deep application of fundamental engineering concepts, algorithmic creativity, and extensive tuning.

---

## 🧩 Team G16

| Name | 
| :--- |
| **Maduka Malruk** | 
| **Narada Madushanka** |
| **Sasidu Madusanka** |

---

## 🎯 Competition Objective

The goal was to build a completely autonomous robot that:
* Navigates a maze built from $22.5\text{cm} \times 22.5\text{cm}$ cells.
* Detects walls in real-time.
* Finds the finish tile (white floor).
* Returns to start and optimizes path over multiple runs.
* Achieves the **shortest completion time** within 3 official trials.

---

## 🛠️ Hardware Components

All components were provided by the Department of Electrical Engineering.

* **Microcontroller:** Arduino Uno
* **Locomotion:** DC Motors ($\times 2$) via Differential Drive
* **Driver:** Motor Driver (H-Bridge)
* **Sensors:**
    * Ultrasonic Sensors ($\times 3$) – Wall detection
    * IR Sensor ($\times 1$) – Finish tile detection
* **Power:** Voltage Regulator
* **Chassis:** Custom 3D-Printed design with Caster Wheel
* **Misc:** Buzzer, standard wires, connectors & breadboard

---

## 🧠 Software & Control Algorithms

### 1. Wall Detection
Real-time distance measurement using 3 ultrasonic sensors. The logic filters false echoes and noise to accurately map immediate surroundings.



### 2. Movement & Motor Control
* Speed and direction control via PWM.
* Calibrated turn angles (Left / Right / $180^\circ$).
* Drift correction for straight-line movement.

### 3. Maze Navigation Logic
* Cell-by-cell traversal.
* Soft-coded **Left / Forward / Right** decision priority.
* Dead-end recovery logic.
* Loop prevention using state flags.

### 4. Finish Tile Detection
Uses an IR sensor to detect the reflective white tile at the goal.

### 5. Path Optimization
The robot is designed to attempt multiple runs, refining its route to reduce unnecessary turns and backtracking.

---

## 🧱 Mechanical & Electronic Design

### 🔩 Chassis
* Fully 3D-printed using UoM lab facilities.
* Supports modular mounting for sensors.
* Lightweight and reinforced for stability.



### ⚡ Circuit Integration
* Clean routing for sensor signals.
* Power system regulated for stable motor performance.
* Noise minimized to avoid false sensor readings.

---
### 🏁 What We LearnedReal
- world sensors behave nothing like simulations.
- Motor tuning without encoders is extremely challenging.
- Small changes in geometry drastically affect movement.
- Simple logic, when tuned well, beats overly complex algorithms.
- Teamwork makes debugging $100\times$ easier.
### 👏 Acknowledgment
- A heartfelt thank-you to:Dr. Thamidu Naveen-Department of Electrical Engineering 
- University of Moratuwa For organizing this highly educational and competitive event and providing all necessary guidance and resources.

### 📬 Connect With Us
- Feel free to reach out or follow our work:
- LinkedIn Post: (Insert link)
- Facebook Album: https://www.facebook.com/share/p/17TBAiJgme/

### ⭐ If you found this repository useful, consider giving it a star!
