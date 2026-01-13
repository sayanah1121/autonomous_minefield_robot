# Autonomous Minefield Robot 
# Autonomous Minefield Mapping & Safe Path Generation

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Python](https://img.shields.io/badge/Language-Python%20%7C%20C%2B%2B-green) ![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205%20%7C%20Arduino-red)

## 🚀 Project Overview
This project implements an autonomous robotic system designed for **defense and humanitarian applications**. The robot navigates an unknown environment, detects hazardous metallic objects (simulating landmines) using an inductive proximity sensor, and constructs a probabilistic digital map of the terrain.

Using the **A* (A-Star) Path Planning algorithm**, the system automatically calculates and visualizes a "Safe Lane"—an optimal, obstacle-free path for troop convoys or follow-up vehicles to traverse safely.

### 🎯 Key Use Case
* **Defense:** Automated reconnaissance of hostile territory to identify IEDs/mines without risking human life.
* **Humanitarian:** Mapping safe routes in post-conflict zones for aid delivery.

---

## 🛠️ System Architecture

The system uses a **Hybrid Compute Architecture**:
* **High-Level Logic (Raspberry Pi 5):** Runs ROS2 (Robot Operating System) to handle Mapping, Path Planning, and State Estimation.
* **Low-Level Control (Arduino):** Handles real-time motor control (PWM), encoder odometry, and sensor data acquisition.

### Tech Stack
* **Middleware:** ROS2 (Humble/Jazzy)
* **Languages:** Python (Nodes), C++ (Arduino Firmware)
* **Algorithms:** * **Mapping:** Bayesian Log-Odds Occupancy Grid
    * **Planning:** A* Search Algorithm (Manhattan Heuristic)
    * **Localization:** Dead Reckoning (Encoder Odometry)

---

## 🧩 Hardware Components

| Component | Specification | Function |
| :--- | :--- | :--- |
| **Main Computer** | Raspberry Pi 5 (8GB) | Runs ROS2 Nodes, Mapper, and Planner |
| **Microcontroller** | Arduino Uno/Nano | Real-time motor & sensor interface |
| **Mine Sensor** | Inductive Proximity Sensor (LJ12A3) | Detects metallic objects (simulated mines) |
| **Drive System** | DC Motors with Encoders | Differential drive with position feedback |
| **Power** | 3S LiPo Battery (11.1V) | Powers motors (12V) and Pi (via Buck Converter) |

---

## 📂 Repository Structure

autonomous_minefield_robot/
├── arduino_firmware/       # C++ code for Arduino
│   └── firmware.ino        # PID Motor Control & Sensor Reading
├── ros2_ws/                # ROS2 Workspace
    └── src/
        └── minefield_bot/  # Custom ROS2 Package
            ├── hardware_bridge.py  # Serial communication driver
            ├── simple_mapper.py    # Probabilistic Grid Mapping
            ├── astar_planner.py    # A* Path Planning Logic
            └── launch/             # Launch files

            
