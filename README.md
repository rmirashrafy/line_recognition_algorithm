# Ackermann Steering Line Recognition System

## Overview

<img width="1189" height="336" alt="Screenshot from 2026-05-08 12-55-42" src="https://github.com/user-attachments/assets/0775cd50-d8e8-4aa5-b2c6-8af62ee64205" />

This project implements a real-world autonomous driving simulation on a 30 × 20 cm robot. A Raspberry Pi with a single camera processes visual input from a printed road banner and controls steering using Ackermann geometry.

The system performs lane-keeping without external sensors such as LiDAR, ultrasonic, or infrared. All processing and decision-making are done onboard in real time.

## System Architecture

### Hardware

* Processor: Raspberry Pi (Raspbian OS)
* Microcontroller: Arduino (connected via serial `/dev/ttyUSB0`, 9600 baud)
* Camera: Picamera2 (30 FPS)
* Steering: Servo motor (Ackermann steering)
* Drive: DC motor controlled via Arduino
* Communication: Serial commands (e.g., `A[angle]`, `F`, `S`)

### Software Layers

* Vision layer: OpenCV processing on Raspberry Pi
* Control layer: PID-based steering control
* Hardware layer: Arduino executes motor and servo commands

## Vision and Preprocessing

To achieve stable performance at 30 FPS:

* Region of Interest (ROI): Hardware cropping focuses only on the lower road area
* Brightness correction: Gamma correction (1.5) reduces glare and improves edge detection stability
* Masking: A polygonal mask isolates the road and removes irrelevant regions

## Line Detection

Lane detection is based on:

* Canny edge detection
* Probabilistic Hough Line Transform (HoughLinesP)

Lines are classified as left or right based on position and slope threshold (|slope| > 2).

To improve robustness:

* Previous detected lines are stored
* If detection fails, last valid lines are reused to maintain direction

<img width="759" height="530" alt="6037193665950106862" src="https://github.com/user-attachments/assets/407f2869-cd6c-4633-82b1-cf15f2f7a244" />

The blue and red lines represent the most probable left and right lane boundaries detected from the set of lines, and the green middle point is the target position that the steering system tries to align with.

## Control System

Steering is controlled using a PID controller.

* Lane center is computed as the midpoint between left and right lanes
* Error is the offset between lane center and image center
* PID output is scaled and limited to match physical steering constraints

Ackermann steering geometry is used to convert control output into realistic wheel angles.

## Communication with Arduino

The Raspberry Pi sends serial commands to Arduino:

* `A[angle]` for steering control
* `F` for forward movement
* `S` for stop

## Key Functions

* Line_Detection: main processing loop for vision and lane detection
* pid_control(error): computes steering correction
* reduce_local_brightness: applies gamma correction and lighting normalization
* send_command: sends serial instructions to Arduino
* average_line: merges multiple detected line segments into stable lane estimates

