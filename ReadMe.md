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
- Precise odometry for turns  
- Flood-fill path planning  
- A 3D-printed chassis designed in the UoM Department of Electrical Engineering  

---

# 🛠️ **Hardware Architecture**

This robot complies with the **Standard Class Micromouse Specifications** defined in the UoM competition guidelines.

| Component | Specification | Purpose |
|----------|--------------|---------|
| **Microcontroller** | Arduino Uno (ATmega328P) | Core processing & control |
| **Sensors** | 3× Ultrasonic (HC-SR04), 1× IR Sensor | Wall detection, finish tile detection |
| **Actuators** | 12V DC Gear Motors | Differential drive locomotion |
| **Motor Driver** | L298N / TB6612FNG | H-bridge speed & direction control |
| **Power** | 3S Li-Po (≤12V) | Main power source |
| **Chassis** | Custom 3D-printed (PLA), 18×18×18 cm limit | Structural assembly |

---

# 🔧 **Software & Algorithms**

### 🔹 **1. PID Wall Following**
Maintains robot centering inside the maze corridor by minimizing the error between left & right distance sensors.

### 🔹 **2. Flood-Fill Maze Solving**
The maze is treated as a grid of 16×16 nodes.

Algorithm steps:
1. Initialize distances from all cells to the goal  
2. Explore and add walls to memory  
3. Recalculate (flood) distances whenever new walls are detected  
4. Move to the adjacent cell with the *lowest distance value*  
5. Once the shortest path is fully mapped → perform **speed-run mode**

### 🔹 **3. Memory Optimization**
- Wall data stored in **bitfields**  
- Fits within Arduino Uno’s **2 KB SRAM**

---


# Team Name: G16
Members:
- Maduka Malruk
- Narada Madushanka
- Sasidu Madusanka

# ❤️ Acknowledgments
-Department of Electrical Engineering – University of Moratuwa
-Competition organizers, lab instructors, and mentors