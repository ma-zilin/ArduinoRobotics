# Arduino Robotics Plan

&gt; From servo PWM to visual servo loops. Phase 1: 2026.04 – 2026.07

[![Arduino](https://img.shields.io/badge/Arduino-Uno-00979D.svg)](https://arduino.cc/)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-IDE-orange.svg)](https://platformio.org/)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)

## Objective

Build the control layer that bridges deep learning perception to physical actuation. Master the "translator" skill: theory → C++ → torque.

## Pipeline

| Project | Duration | Core Skill | Deliverable |
|---------|----------|------------|-------------|
| **A: 2D LiDAR** | 2 weeks | PWM + serial | Polar→Cartesian point cloud |
| **B: Self-Balancing Gimbal** | 2–3 weeks | **PID tuning** | IMU-stabilized platform |
| **C: Visual Servo** | 2 weeks | System integration | YOLOv8 → servo tracking |

## Hardware

| Component | Qty | Reuse Path |
|-----------|-----|------------|
| Arduino Uno | 1 | A → B → C |
| SG90 servo | 2 | A → B → C (hot-swap) |
| HC-SR04 | 1 | Project A only |
| MPU6500 (I2C) | 1 | Project B only |
| Laptop camera | 1 | Project C (zero cost) |

**Budget:** &lt; ¥150 (if basic kit owned)

## Project C: Visual Servo (Capstone)

**Architecture:** Laptop perception + Arduino execution
┌─────────────┐      Serial       ┌─────────────┐
│   Laptop    │ ◄────────────────►│   Arduino   │
│  YOLOv8     │  angle_cmd        │  PWM driver │
│  Python PID │                   │  SG90 servo │
└─────────────┘                   └─────────────┘

**Flow:** `cv2.VideoCapture` → YOLOv8 inference → pixel error → PID → serial → servo angle

## Tech Stack

- **IDE:** VS Code + PlatformIO
- **Standard:** `src/main.cpp` (no `.ino`), per-project `platformio.ini`
- **Repo:** `arduino-robotics-plan`, branch `dev/project-x` → `main`

## Skill Progression
A: Open-loop        B: Closed-loop         C: AI-in-the-loop
│                    │                      │
├─ Actuator control  ├─ Sensor fusion       ├─ Deep learning
├─ Basic comms       ├─ Embedded PID        ├─ Cross-language protocol
└─ Geometry          └─ Physical intuition   └─ System integration


## Checklist

- [ ] Project A: servo scan + ultrasonic trigger + serial output
- [ ] Project A: Python point cloud visualization
- [ ] Project B: MPU6500 I2C read + smoothing
- [ ] **Project B: Hand-written C++ PID, tuned to critical damping**
- [ ] Project B: Document Kp/Ki/Kd tuning log
- [ ] Project C: YOLOv8-nano ≥ 15 FPS local inference
- [ ] Project C: Python PID → servo angle mapping
- [ ] Global: Wiring diagrams + demo GIFs per project

## Strategic Value

| Phase | This Repo Enables |
|-------|-------------------|
| Phase 2 (Isaac Lab) | Deploy RL policies to real hardware |
| Phase 3 (VLA) | Edge interface for vision-language-action models |
| Lab Entry | Demonstrated control + perception integration |

## The Pitch

> *"Because I don't just write elegant AI architectures—I've tuned motor PID controllers and I know what real-world friction looks like."*

— Summer 2028, when asked why a software engineer belongs in a robotics lab.

## Next Step

Choose your entry point:
1. `project-a-2d-lidar/src/main.cpp` — start scanning
2. PlatformIO setup guide — configure environment