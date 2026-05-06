
# Lane Recognition & Autonomous Navigation System
### 1/10 Scale Ackermann Robot

## Project Overview
This project implements a computer vision-based navigation system for a 1/10 scale autonomous vehicle. The goal is to achieve stable **lane-keeping** on a printed road banner using a single camera sensor. The system simulates real-world automotive steering geometry (Ackermann) and performs all processing locally on a mobile microprocessor.

## Hardware Stack
*   **Processing Unit:** Raspberry Pi running Raspbian OS.
*   **Visual Sensor:** Camera module (primary and only sensor input).
*   **Actuation:** Servo motor controlled via **GPIO PWM** to adjust steering angles.
*   **Chassis:** 4-wheel robot with **Ackermann steering geometry**, providing a realistic turning radius and wheel alignment similar to a full-sized car.

## Computer Vision Pipeline

### 1. Image Preprocessing
*(To be completed with specific isolation methods such as HSV Masking or Grayscale/Canny.)*

### 2. Region of Interest (ROI) Optimization
To maximize processing efficiency and reduce latency, the algorithm utilizes a specialized "Cut Part" technique:
*   **Focus:** The input frame is cropped to focus exclusively on the bottom portion of the camera feed.
*   **Noise Reduction:** By limiting the search area, the system avoids "phantom lines" or parallel roads printed elsewhere on the map banner.
*   **Performance:** Reducing the pixel count significantly lowers the computational load on the Raspberry Pi CPU.

### 3. Line Detection Logic
*(To be completed with specific detection methods such as Hough Transform or Sliding Window.)*

## Control System (The Brain)

### PID Steering Control
The navigation logic uses a **PID (Proportional-Integral-Derivative) controller** to translate visual data into steering commands.
*   **Error Calculation:** The algorithm calculates the offset between the robot’s current heading and the center of the detected lane.
*   **Output:** The PID process generates an angle command, which is sent directly to the servo motor via GPIO to correct the robot's trajectory.

### Speed Management
*(Current Status: Development in progress / Constant speed default.)*
