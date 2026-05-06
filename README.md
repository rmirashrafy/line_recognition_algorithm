Ackermann Steering Line Recognition System
Autonomous Vehicle Navigation with OpenCV & Raspberry Pi

This project implements a real-world autonomous driving simulation scaled down to a 30 cm X 20 cm robot. Using a **Raspberry Pi** as the primary processing unit and a single camera sensor, the vehicle identifies lane boundaries on a printed banner and navigates using **Ackermann steering geometry**.

Project Overview
The core challenge of this project is to achieve stable **lane-keeping** without any external sensors (no LiDAR, ultrasonic, or infrared). All decision-making happens on-board, processing high-resolution visual data in real-time to control a servo motor for steering and an Arduino-linked motor for propulsion.

Hardware Architecture

* **Processor:** Raspberry Pi (Running Raspbian OS).
* **Microcontroller:** Arduino (Connected via `/dev/ttyUSB0` at 9600 baud) for low-level motor and servo execution.
* **Camera:** Picamera2 (Capturing at 30 FPS).
* **Actuators:**
    * **Steering:** Servo motor (Ackermann geometry).
    * **Drive:** DC Motor (Controlled via Arduino Serial commands).
* **Communication:** Serial protocol sends commands like `A[angle]` for steering and `F`/`S` for forward/stop.

Software Pipeline

### 1. Vision & Preprocessing
To maintain 30 FPS on a Raspberry Pi, the code uses a multi-stage optimization:
* **Hardware Cropping (ROI):** The camera sensor doesn't capture the whole room; it uses `ScalerCrop` to focus only on a $2800 \times 1000$ pixel area at the bottom of the road.
* **Gamma Correction:** The `reduce_local_brightness` function applies a gamma of $1.5$. This prevents light reflections on the banner from "blinding" the edge detection.
* **Masking:** A polygonal mask further isolates the road, ignoring lines from neighboring lanes or map artifacts.

### 2. Line Detection Logic
The algorithm uses a **Canny Edge Detection** and **Probabilistic Hough Line Transform** (`HoughLinesP`).
* **Line Classification:** Lines are categorized as **Left** or **Right** based on their position relative to the image center and a slope filter (slopes must be $> 2$ to be considered valid vertical-ish road lines).
* **Persistence:** If the camera loses a line temporarily, the code stores the `last_left_lines` or `last_right_lines` as a fallback to prevent the car from losing direction.

### 3. Navigation Control (The "Brain")
The steering is governed by a **PID Controller** ($K_p=0.7, K_i=0.0, K_d=0.0$):
* **Error Calculation:** The "Center Point" is calculated as the midpoint between the left and right lane averages. The error is the distance between this midpoint and the horizontal center of the car.
* **Command Scaling:** The PID output is clipped and mapped to a steering range, then halved to match the physical limits of the 1/10 scale steering rack.

Code Structure & Key Functions

| Function | Purpose |
| :--- | :--- |
| `Line_Detection()` | The main loop function. Handles edge detection, ROI masking, and calculating the steering angle. |
| `pid_control(error)` | Computes the correction value based on the proportional deviation from the lane center. |
| `reduce_local_brightness()` | Normalizes the image lighting using HSV color space and gamma power functions. |
| `send_command()` | Encodes and sends serial strings to the Arduino (e.g., `A45` for a 45-degree turn). |
| `average_line()` | Takes multiple Hough line segments and collapses them into a single stable coordinate set. |



Autonomous Lane-Keeping Robot (1/10 Scale)
## Documentation

### 1. System Architecture
The project is built on a distributed control model:
* **Vision Layer (Raspberry Pi):** Uses a `Picamera2` to capture high-resolution frames, which are then cropped and processed via **OpenCV** to identify lane boundaries.
* **Control Layer (PID):** Calculates the error based on the deviation of the lane center from the image center.
* **Hardware Layer (Arduino):** Receives steering angles (`A{angle}`) and movement commands (`F`, `S`) via Serial to drive the motors and servos.

### 2. Computer Vision Pipeline
To handle the limited resources of the Raspberry Pi while maintaining accuracy:
* **Dynamic ROI (Region of Interest):** The camera uses a `ScalerCrop` at the hardware level to focus on the road, followed by a **polygonal mask** in the code to ignore off-road artifacts.
* **Brightness Normalization:** A custom `reduce_local_brightness` function uses Gamma correction to prevent "washout" from overhead lights on the banner.
* **Color-Agnostic Detection:** It uses **Canny Edge Detection** combined with a **Hough Line Transform** to find structural lines, but also includes an **HSV-based red pixel mean tracker** as a fallback or secondary indicator.

### 3. Navigation & Control
* **Lane Midpoint:** The algorithm averages the slopes of detected left and right lines to find a central trajectory.
* **Ackermann Logic:** The steering angle is calculated using a PID formula:
    $$Control = K_p \cdot e + K_i \cdot \int e \, dt + K_d \cdot \frac{de}{dt}$$
* **Fail-safes:** If a line is lost, the system uses the `last_detected_lines` to prevent sudden steering "jerks."

  
