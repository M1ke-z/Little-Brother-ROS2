# Quadruped Robot

A ROS 2 powered 12-DOF quadruped robot built using hobbyist PWM servos, a Raspberry Pi 4B, and a custom lightweight plastic frame. The project focuses on gait generation, robot stability, motion control, and real-world embedded robotics using ROS 2 Jazzy running inside Docker.

## Robot Overview

### Hardware Specifications

- **12x 30kg PWM Servos**
  - 3 DOF per leg
  - Hobbyist-grade high torque servos

- **Frame**
  - Lightweight plastic frame
  - Modular leg structure

- **Controller**
  - Raspberry Pi 4B (4GB RAM)

- **PWM Driver**
  - PCA9685 16-channel PWM controller

- **Battery**
  - 5200 mAh 2S LiPo battery

- **Software Stack**
  - ROS 2 Jazzy
  - Docker-based development environment

---

## Features

- ROS 2 based robot architecture
- Modular gait generation system
- Bézier curve based foot trajectories
- Support for multiple walking gaits
- PWM servo control through PCA9685
- Dockerized development workflow
- Gazebo simulation support
- IMU integration for balancing and stability control
- Extensible node-based control architecture

---

## Project Structure

```text
src/
├── control_pkg/
├── interfaces/
├── launch_pkg/
├── balance_pkg/
├── pca9685_driver/
└── initialize.sh
README.md
.gitignore
```

---

## Software Architecture

The robot software is organized into several ROS 2 nodes:

| Node | Purpose |
|---|---|
| Control Node | Handles gait execution and joint commands |
| Balance Node | Processes IMU data for stabilization |
| PCA9685 | Communicates with PCA9685 and servos |

---

## Gait System

The quadruped uses a phase-based gait controller with configurable:

- Gait frequencies
- Phase offsets
- Swing and stance trajectories
- Bézier control points
- Interpolated gait transitions

Supported and planned gaits include:

- Stand
- Walk
- Trot
- Turning motions
- Dynamic gait transitions

---

## Simulation

The project includes Gazebo simulation support using ROS 2 control interfaces.

Simulation features include:

- Joint trajectory control
- IMU simulation
- Robot state publishing
- Controller manager integration
- Real-time gait testing

---

## Development Goals

This project is intended as both:

- A research and learning platform for quadruped locomotion
- A fully functional embedded robotics system

Areas of focus include:

- Robotics software architecture
- Control systems
- Embedded Linux
- ROS 2
- Real-time motion control
- Simulation and testing
- Hardware/software integration

---

## License

This project is licensed under the MIT License.
