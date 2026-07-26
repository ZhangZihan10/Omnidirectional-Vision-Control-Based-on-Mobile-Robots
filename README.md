# Omnidirectional Vision-Based Obstacle Avoidance for Mobile Robots

This repository implements a **closed-loop obstacle avoidance system for a Mecanum-wheeled mobile robot** based on **MATLAB–Unity co-simulation**.

The proposed system integrates:

- Omnidirectional vision perception
- Structured-light ranging
- Dynamic occupancy-grid mapping
- Local RRT* path replanning
- Eight-direction trajectory generation
- Feedforward fuzzy adaptive PID control

The framework establishes a complete **perception–mapping–planning–control–feedback** pipeline for autonomous mobile robot navigation.

---

# System Overview

The proposed framework combines virtual simulation and physical experiments to achieve real-time obstacle avoidance for an omnidirectional mobile robot.

The system consists of:

- MATLAB-based computation and control layer
- Unity-based virtual robot simulation environment
- Omnidirectional vision sensing module
- Structured-light obstacle measurement module
- Dynamic occupancy-grid mapping module
- Local path planning and trajectory tracking module
- Real Mecanum-wheel robot platform

---

# Workflow

The obstacle avoidance framework follows a perception–planning–control workflow:

### 1. Laser Stripe Extraction

Extract laser stripes from omnidirectional camera images.

### 2. Obstacle Position Estimation

Calculate obstacle positions using structured-light ranging.

### 3. Dynamic Occupancy-Grid Mapping

Update the environment map according to detected obstacles.

### 4. Collision Detection

Detect potential collisions along the remaining trajectory.

### 5. Local RRT* Replanning

Trigger local RRT* path replanning when collision risks are detected.

### 6. Trajectory Generation

Generate executable eight-direction trajectories for the Mecanum-wheel robot.

### 7. Motion Control

Calculate four-wheel velocities and generate PWM control commands.

### 8. Feedback Correction

Apply fuzzy adaptive PID control for trajectory tracking.

---

# Requirements

## Software

- MATLAB
- Unity

## MATLAB Toolboxes

- Image Processing Toolbox
- MATLAB Support Package for Arduino Hardware

## Hardware

- Omnidirectional camera or fisheye camera
- Line-laser module
- Four-wheel Mecanum mobile robot
- Arduino-based motor controller

## Calibration Data

The following calibration files are required:

- Camera calibration parameters
- Laser-plane calibration parameters
- Robot coordinate transformation parameters

---

# Usage

## MATLAB–Unity Co-simulation

Run:

```matlab
ARealTimeTest8_5