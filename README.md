\# Omnidirectional Vision-Based Obstacle Avoidance for Mobile Robots



This repository implements a \*\*closed-loop obstacle avoidance system for a Mecanum-wheeled mobile robot\*\* based on \*\*MATLAB–Unity co-simulation\*\*.



The proposed system integrates:



\- Omnidirectional vision perception

\- Structured-light ranging

\- Dynamic occupancy-grid mapping

\- Local RRT\* path replanning

\- Eight-direction trajectory generation

\- Feedforward fuzzy adaptive PID control



The framework establishes a complete \*\*perception–mapping–planning–control–feedback\*\* pipeline for autonomous mobile robot navigation.



\---



\## System Overview



The system combines virtual simulation and physical experiments to achieve real-time obstacle avoidance for an omnidirectional mobile robot.



Main components include:



\- MATLAB-based computation and control layer

\- Unity-based virtual robot simulation environment

\- Omnidirectional vision sensing module

\- Structured-light obstacle measurement module

\- Dynamic environment mapping module

\- Local path planning and trajectory tracking module

\- Real Mecanum-wheel robot platform



\---



\## Workflow



The proposed obstacle avoidance framework follows the following workflow:



1\. \*\*Laser stripe extraction\*\*

&#x20;  

&#x20;  Extract laser stripes from omnidirectional camera images.



2\. \*\*Obstacle position estimation\*\*



&#x20;  Calculate obstacle positions using structured-light ranging.



3\. \*\*Dynamic occupancy-grid mapping\*\*



&#x20;  Update the environmental map according to detected obstacles.



4\. \*\*Collision detection\*\*



&#x20;  Detect potential collisions along the remaining trajectory.



5\. \*\*Local path replanning\*\*



&#x20;  Trigger local RRT\* replanning when collision risks are detected.



6\. \*\*Trajectory generation\*\*



&#x20;  Generate executable eight-direction trajectories for the Mecanum-wheel robot.



7\. \*\*Motion control\*\*



&#x20;  Calculate four-wheel velocities and generate PWM control commands.



8\. \*\*Feedback correction\*\*



&#x20;  Apply fuzzy adaptive PID control for trajectory tracking.



\---



\# Requirements



\## Software



\- MATLAB

\- Unity



\## MATLAB Toolboxes



\- Image Processing Toolbox

\- MATLAB Support Package for Arduino Hardware



\## Hardware



\- Omnidirectional camera or fisheye camera

\- Line-laser module

\- Four-wheel Mecanum mobile robot

\- Arduino-based motor controller



\## Calibration Data



The following calibration files are required:



\- Camera calibration parameters

\- Laser-plane calibration parameters

\- Robot coordinate transformation parameters



\---



\# Usage



\## 1. MATLAB–Unity Co-simulation



Run:



```matlab

ARealTimeTest8\_5



\# Physical Robot Experiment



The proposed obstacle avoidance system was further validated on a real Mecanum-wheeled mobile robot platform.



The experimental setup consists of:



\- Mecanum-wheel mobile robot

\- Omnidirectional vision camera

\- Line-laser sensing module

\- MATLAB-based real-time control framework





\## Real-world Experimental Demonstration



The robot performs autonomous obstacle detection and local trajectory adjustment in an indoor environment.



<p align="center">

&#x20; <img src="IMG\_0699.gif" width="600">

</p>



\*\*Fig. Real-world obstacle avoidance experiment using the Mecanum-wheeled mobile robot.\*\*





