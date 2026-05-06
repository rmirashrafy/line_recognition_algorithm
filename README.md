
# 🏎️ Ackermann Steering Line Recognition System
### Autonomous 1/10 Scale Vehicle Navigation with OpenCV & Raspberry Pi

This project implements a real-world autonomous driving simulation scaled down to 1/10. Using a **Raspberry Pi** as the primary processing unit and a single camera sensor, the vehicle identifies lane boundaries on a printed banner and navigates using **Ackermann steering geometry**.

## 📌 Project Overview
The core challenge of this project is to achieve stable **lane-keeping** without any external sensors (no LiDAR, ultrasonic, or infrared). All decision-making happens on-board, processing high-resolution visual data in real-time to control a servo motor for steering and an Arduino-linked motor for propulsion.

---

## 🛠 Hardware Architecture

*   **Processor:** Raspberry Pi (Running Raspbian OS).
*   **Microcontroller:** Arduino (Connected via `/dev/ttyUSB0` at 9600 baud) for low-level motor and servo execution.
*   **Camera:** Picamera2 (Capturing at 30 FPS).
*   **Actuators:** 
    *   **Steering:** Servo motor (Ackermann geometry).
    *   **Drive:** DC Motor (Controlled via Arduino Serial commands).
*   **Communication:** Serial protocol sends commands like `A[angle]` for steering and `F`/`S` for forward/stop.

---

## 💻 Software Pipeline

### 1. Vision & Preprocessing
To maintain 30 FPS on a Raspberry Pi, the code uses a multi-stage optimization:
*   **Hardware Cropping (ROI):** The camera sensor doesn't capture the whole room; it uses `ScalerCrop` to focus only on a $2800 \times 1000$ pixel area at the bottom of the road.
*   **Gamma Correction:** The `reduce_local_brightness` function applies a gamma of $1.5$. This prevents light reflections on the banner from "blinding" the edge detection.
*   **Masking:** A polygonal mask further isolates the road, ignoring lines from neighboring lanes or map artifacts.

### 2. Line Detection Logic
The algorithm uses a **Canny Edge Detection** and **Probabilistic Hough Line Transform** (`HoughLinesP`).
*   **Line Classification:** Lines are categorized as **Left** or **Right** based on their position relative to the image center and a slope filter (slopes must be $> 2$ to be considered valid vertical-ish road lines).
*   **Persistence:** If the camera loses a line temporarily, the code stores the `last_left_lines` or `last_right_lines` as a fallback to prevent the car from losing direction.

### 3. Navigation Control (The "Brain")
The steering is governed by a **PID Controller** ($K_p=0.7, K_i=0.0, K_d=0.0$):
*   **Error Calculation:** The "Center Point" is calculated as the midpoint between the left and right lane averages. The error is the distance between this midpoint and the horizontal center of the car.
*   **Command Scaling:** The PID output is clipped and mapped to a steering range, then halved to match the physical limits of the 1/10 scale steering rack.

---

## 📂 Code Structure & Key Functions

| Function | Purpose |
| :--- | :--- |
| `Line_Detection()` | The main loop function. Handles edge detection, ROI masking, and calculating the steering angle. |
| `pid_control(error)` | Computes the correction value based on the proportional deviation from the lane center. |
| `reduce_local_brightness()` | Normalizes the image lighting using HSV color space and gamma power functions. |
| `send_command()` | Encodes and sends serial strings to the Arduino (e.g., `A45` for a 45-degree turn). |
| `average_line()` | Takes multiple Hough line segments and collapses them into a single stable coordinate set. |

---

## 🚀 Getting Started

### Prerequisites
*   Raspberry Pi with `Picamera2` library installed.
*   OpenCV (`cv2`), `numpy`, and `pyserial`.
*   Arduino flashed to accept serial commands (e.g., parsing `A` for steering and `F` for speed).

### Installation
1.  Connect the Arduino to the Pi via USB.
2.  Ensure your camera is enabled in `raspi-config`.
3.  Run the main script:
    ```bash
    python line_recognition.py
    ```

### Controls
*   **'q':** Stops the vehicle (sends 'S' command) and exits the program.
This is a solid implementation of a vision-based control loop. You’ve combined high-level image processing (Hough Lines, ROI masking, and brightness gamma correction) with a low-level PID controller communicating over Serial.

Below is the **extended and restructured documentation** for your project, followed by the **finalized code** with some minor syntax cleanups (like moving the misplaced `import` and fixing logic indentation) to ensure it runs smoothly.

---

# Autonomous Lane-Keeping Robot (1/10 Scale)
## Documentation

### 1. System Architecture
The project is built on a distributed control model:
*   **Vision Layer (Raspberry Pi):** Uses a `Picamera2` to capture high-resolution frames, which are then cropped and processed via **OpenCV** to identify lane boundaries.
*   **Control Layer (PID):** Calculates the error based on the deviation of the lane center from the image center.
*   **Hardware Layer (Arduino):** Receives steering angles (`A{angle}`) and movement commands (`F`, `S`) via Serial to drive the motors and servos.

### 2. Computer Vision Pipeline
To handle the limited resources of the Raspberry Pi while maintaining accuracy:
*   **Dynamic ROI (Region of Interest):** The camera uses a `ScalerCrop` at the hardware level to focus on the road, followed by a **polygonal mask** in the code to ignore off-road artifacts.
*   **Brightness Normalization:** A custom `reduce_local_brightness` function uses Gamma correction to prevent "washout" from overhead lights on the banner.
*   **Color-Agnostic Detection:** It uses **Canny Edge Detection** combined with a **Hough Line Transform** to find structural lines, but also includes an **HSV-based red pixel mean tracker** as a fallback or secondary indicator.

### 3. Navigation & Control
*   **Lane Midpoint:** The algorithm averages the slopes of detected left and right lines to find a central trajectory.
*   **Ackermann Logic:** The steering angle is calculated using a PID formula:
    $$Control = K_p \cdot e + K_i \cdot \int e \, dt + K_d \cdot \frac{de}{dt}$$
*   **Fail-safes:** If a line is lost, the system uses the `last_detected_lines` to prevent sudden steering "jerks."

---

## Final Project Code

```python
import cv2
import numpy as np
import time
import math
import serial
from picamera2 import Picamera2

# --- Configuration & Hyperparameters ---
raw_size = (3200, 2400)
crop_params = (120, 2400, 2800, 1000)  # (x, y, w, h)
output_size = (800, 600)

# PID Constants
Kp, Ki, Kd = 0.7, 0.0, 0.0
prev_error = 0
integral = 0

motor_speed = 100
move_command = 'F'
last_left_lines = None
last_right_lines = None

# --- Hardware Initialization ---
try:
    arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2) 
except Exception as e:
    print(f"Error: Could not connect to Arduino: {e}")
    exit()

def send_command(command):
    try:
        msg = f"{command}\n".encode()
        arduino.write(msg)
    except Exception as e:
        print(f"Error: Failed to send command: {e}")

# Initial Setup
send_command(str(motor_speed))
time.sleep(1)
send_command(move_command)

# Camera Setup
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": output_size, "format": "RGB888"},
    raw={"size": raw_size},
    controls={"ScalerCrop": crop_params, "FrameRate": 30})
picam2.configure(config)
picam2.start()

cv2.namedWindow("line detection", cv2.WINDOW_NORMAL)
cv2.resizeWindow("line detection", 640, 480)

# --- Helper Functions ---

def find_white_mean_x(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 0, 200])   
    upper_red = np.array([180, 40, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    red_pixels = np.column_stack(np.where(mask > 0))
    if red_pixels.size == 0:
        return 1000
    return np.mean(red_pixels[:, 1])

def reduce_local_brightness(image, threshold=1, gamma=1.0, kernel_size=10):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    _, bright_mask = cv2.threshold(v, threshold, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_CLOSE, kernel)
    v_norm = v / 255.0
    v_gamma = np.uint8(np.power(v_norm, gamma) * 255)
    v_adjusted = v.copy()
    v_adjusted[bright_mask == 255] = v_gamma[bright_mask == 255]
    return cv2.cvtColor(cv2.merge([h, s, v_adjusted]), cv2.COLOR_HSV2BGR)

def pid_control(error):
    global prev_error, integral
    integral += error
    derivative = error - prev_error
    prev_error = error
    return Kp * error + Ki * integral + Kd * derivative

def average_line(lines):
    if not lines or len(lines) == 0:
        return None
    x1_avg = sum(line[0][0] for line in lines) // len(lines)
    y1_avg = sum(line[0][1] for line in lines) // len(lines)
    x2_avg = sum(line[0][2] for line in lines) // len(lines)
    y2_avg = sum(line[0][3] for line in lines) // len(lines)
    return [[x1_avg, y1_avg, x2_avg, y2_avg]]

def Line_Detection(img):
    global last_left_lines, last_right_lines
    height, width = img.shape[:2]
    
    # Preprocessing
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    
    # Mask ROI
    mask = np.zeros_like(edges)
    polygon = np.array([[ (50, 370), (750, 370), (500, 0), (300, 0)]], np.int32)
    cv2.fillPoly(mask, [polygon], 255)
    masked_edges = cv2.bitwise_and(edges, mask)
    
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 50, minLineLength=100, maxLineGap=100)

    left_lines, right_lines = [], []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0: continue
            slope = abs((y2 - y1) / (x2 - x1))
            if slope > 2:
                if x1 < width // 2:
                    left_lines.append(line)
                else:
                    right_lines.append(line)
        
        # Update persistence
        if left_lines: last_left_lines = left_lines
        if right_lines: last_right_lines = right_lines
    
    avg_left = average_line(left_lines or last_left_lines)
    avg_right = average_line(right_lines or last_right_lines)

    # Calculate Steering
    servo_angle = 90 # Default center
    if avg_left and avg_right:
        l_mid = (avg_left[0][0] + avg_left[0][2]) // 2
        r_mid = (avg_right[0][0] + avg_right[0][2]) // 2
        center_point = (l_mid + r_mid) // 2
        error = center_point - (width // 2)
        
        steering_output = pid_control(error)
        # Scaling and constraints for your specific hardware
        servo_angle = int(np.clip((steering_output * 1.1) + 110, 0, 180) / 2)
        
        # Visuals
        cv2.circle(img, (center_point, height//2), 5, (0, 255, 0), -1)
    elif avg_right:
        servo_angle = 30 # Hard turn if left line lost
        
    return servo_angle

# --- Main Loop ---
try:
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) 
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = reduce_local_brightness(frame, gamma=1.5)

        angle = Line_Detection(frame)
        if angle is not None:
            send_command(f"A{angle}")
        
        cv2.imshow('line detection', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    send_command('S')
    picam2.stop()
    cv2.destroyAllWindows()
```

### Final Question:
In your `Line_Detection` function, when only the **right line** is found, you set the angle to `30`. Is that enough to bring the car back to center, or would you like to implement a "Searching" behavior where the car turns more sharply?

## ⚠️ Known Constraints
*   **Lane Loss:** If only the right line is detected, the car is programmed to default to a sharp left turn (`servo_angle = 30`) to re-enter the lane.
*   **Lighting:** Significant changes in environment lighting may require tuning the `gamma` variable in the `reduce_local_brightness` function.
```
Thanks for those details. It sounds like you are building a focused, efficient lane-keeping system where processing speed is the priority. 

Since you want to keep the **Preprocessing** and **Line Detection** sections empty for now (likely as placeholders for the specific algorithms you choose later), I have structured the documentation to emphasize the **Hardware Integration**, the **Region of Interest (ROI)** logic, and the **Ackermann Steering** physics.

Here is the restructured and extended version of your project text:

---

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

---

To help me fill in the remaining gaps later, I have a few more specific questions:

1.  **Steering Precision:** Since you are using a Raspberry Pi's GPIO for PWM, are you using a specific library (like `Rpi.GPIO` or `pigpio`) to handle the servo jitter?
2.  **Lane Center:** For the "Lane Keeping" aspect, does your code calculate a "midpoint" between two detected lines, or does it try to follow the boundaries?
3.  **The "Cut Part" (ROI):** Is this a static crop (fixed coordinates), or does the crop area move depending on the turn angle?

How would you like to define these sections?
