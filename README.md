
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

---

## ⚠️ Known Constraints
*   **Lane Loss:** If only the right line is detected, the car is programmed to default to a sharp left turn (`servo_angle = 30`) to re-enter the lane.
*   **Lighting:** Significant changes in environment lighting may require tuning the `gamma` variable in the `reduce_local_brightness` function.
```
