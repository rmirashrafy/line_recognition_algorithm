import cv2
import numpy as np
import time
import math
import serial
from picamera2 import Picamera2

raw_size = (3200, 2400)
crop_params = (120,2400, 2800, 1000)  # (x, y, w, h)
output_size = (800, 600)

#main file including PID control
Kp = 0.4
Ki = 0.0
Kd = 0.0
prev_error = 0
integral = 0

motor_speed = 0      # مقدار سرعت موتور (0 تا 255)
move_command = 'F' 

# اتصال به آردوینو
try:
    arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2)  # صبر برای راه‌اندازی کامل آردوینو
except Exception as e:
    print(f"Error: Could not connect to Arduino: {e}")
    exit()
    

def send_command(command):
    try:
        command += '\n'  # اضافه کردن کاراکتر newline برای تکمیل دستور
        arduino.write(command.encode())
        print(f"Sent: {command.strip()}")
    except Exception as e:
        print(f"Error: Failed to send command: {e}")

    # ارسال دستور تنظیم سرعت موتور
send_command(str(motor_speed))
time.sleep(2)
    # ارسال فرمان حرکت (مثلاً حرکت به جلو)
send_command(move_command)
time.sleep(2)
picam2 = Picamera2()
config = picam2.create_preview_configuration(
main={"size": output_size, "format": "RGB888"},
raw={"size": raw_size},
controls={"ScalerCrop": crop_params, "FrameRate": 10})
picam2.configure(config)
picam2.start()

'''cv2.namedWindow("window", cv2.WINDOW_NORMAL)
cv2.resizeWindow("window", 640, 480 ) 

cv2.namedWindow("canny", cv2.WINDOW_NORMAL)
cv2.resizeWindow("canny", 640, 480)

cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Mask", 640, 480)  '''

cv2.namedWindow("line detection", cv2.WINDOW_NORMAL)
cv2.resizeWindow("line detection", 640, 480)


def reduce_local_brightness(image, threshold=1, gamma=1.0, kernel_size=10):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    _, bright_mask = cv2.threshold(v, threshold, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_CLOSE, kernel)
    v_norm = v / 255.0
    v_gamma = np.power(v_norm, gamma)
    v_gamma = np.uint8(v_gamma * 255)
    v_adjusted = v.copy()
    v_adjusted[bright_mask == 255] = v_gamma[bright_mask == 255]
    hsv_adjusted = cv2.merge([h, s, v_adjusted])
    return cv2.cvtColor(hsv_adjusted, cv2.COLOR_HSV2BGR)
def nothing(x):
    pass

# ایجاد Trackbarها با مقادیر پیش‌فرض مناسب
'''cv2.namedWindow("Trackbars")
cv2.createTrackbar("L - H", "Trackbars", 0, 180, nothing)  # Hue بین ۰ تا ۱۸۰
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 200, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 180, 180, nothing)  # Hue بین ۰ تا ۱۸۰
cv2.createTrackbar("U - S", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)'''

def pid_control(error):
    print(f'error: {error}')
    global prev_error, integral
    integral += error
    derivative = error - prev_error
    prev_error = error
    control = Kp * error + Ki * integral + Kd * derivative
    return control

def Line_Detection(img):
    height, width = img.shape[:2]
    
    
    mask = np.zeros_like(img)
    polygon = np.array([[ (70, 500), (750,500), (750 , 50), (70, 50 )]], np.int32)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)  # Apply Gaussian Blur to reduce noise
    mask = np.zeros_like(blur) 
    cv2.fillPoly(mask, [polygon], (255, 255, 255))
  # Edge detection
    edges = cv2.Canny(blur, 50, 150)
    edges = cv2.bitwise_and(edges, mask)  # Apply the mask to the image
    cv2.polylines(img, [polygon], isClosed=True, color=(0, 0, 0), thickness=2)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=40, maxLineGap=100)

    left_lines, right_lines = [], []
    if lines is not None:
        
        for line in lines:
            
            x1, y1, x2, y2 = line[0]
            slope=abs((y2-y1)//(x2-x1))
            #arz = x2 - x1
            if x1 < width // 2 and x2 < width // 2 and (slope>5):
                left_lines.append(line)
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            elif x1 > width // 2 and x2 > width // 2 and (slope>5):
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                right_lines.append(line)

    def average_line(lines):
        if not lines:
            return None
        x1_avg = sum(line[0][0] for line in lines) // len(lines)
        y1_avg = sum(line[0][1] for line in lines) // len(lines)
        x2_avg = sum(line[0][2] for line in lines) // len(lines)
        y2_avg = sum(line[0][3] for line in lines) // len(lines)
        return [[x1_avg, y1_avg, x2_avg, y2_avg]]

    avg_left_line = average_line(left_lines)
    avg_right_line = average_line(right_lines)

    if avg_left_line:
        x1, y1, x2, y2 = avg_left_line[0]
        cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 30)

    if avg_right_line:
        x1, y1, x2, y2 = avg_right_line[0]
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 30)

    # نقطه میانی مسیر
    if avg_left_line and avg_right_line:
        left_center = ((avg_left_line[0][0] + avg_left_line[0][2]) // 2, (avg_left_line[0][1] + avg_left_line[0][3]) // 2)
        right_center = ((avg_right_line[0][0] + avg_right_line[0][2]) // 2, (avg_right_line[0][1] + avg_right_line[0][3]) // 2)
        center_point = ((left_center[0] + right_center[0]) // 2, (left_center[1] + right_center[1]) // 2)
    elif avg_left_line:
        center_point = ((avg_left_line[0][0] + avg_left_line[0][2]) // 2, (avg_left_line[0][1] + avg_left_line[0][3]) // 2)
    elif avg_right_line:
        center_point = ((avg_right_line[0][0] + avg_right_line[0][2]) // 2, (avg_right_line[0][1] + avg_right_line[0][3]) // 2)
    else:
        center_point = None

    if center_point:
        cv2.circle(img, center_point, 5, (0, 255, 0), 100)
        
        car_center_x = width // 2
        #print(f"car center:{car_center_x}")
        car_center_x =car_center_x
        l=center_point[0]
        #print(f"center_point : {center_point[0]}")
        error = (l - car_center_x)
        
        #print(f"error: {error}")
        
        steering_angle = int(np.clip(pid_control(error), -100, 100))
        print(f'Steering Angle: {steering_angle}')
        steering_angle = ((steering_angle*1.1)+110)
        if steering_angle==0:
           steering_angle = 50
        else:
            steering_angle=steering_angle/2
        servo_angle = int(steering_angle)

        #print(f'Steering Angle: {servo_angle}')
        return servo_angle


while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) 
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    frame = reduce_local_brightness(frame, threshold=1, gamma=1.5, kernel_size=10)
    

    '''hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")
    lower = np.array([l_h, l_s, l_v])
    upper = np.array([u_h, u_s, u_v])
    
    mask = cv2.inRange(hsv, lower, upper)
    edges = cv2.Canny(mask, 50, 150)'''
    
    
    servo_angle = 45  # مقدار پیش‌فرض در صورت عدم تشخیص خط
    temp_angle = Line_Detection(frame)
    if temp_angle is not None:
      servo_angle = temp_angle
      send_command(f"A{servo_angle}")
      
    '''cv2.imshow('mask', mask)  
    cv2.imshow('canny', edges)
    #cv2.imshow('Original Video', image)
    ''' 
    cv2.imshow('line detection', frame)  
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        
        move_command = 'S'
        send_command(move_command)
        time.sleep(2) 
        cv2.destroyAllWindows()
        break

