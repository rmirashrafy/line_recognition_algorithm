import cv2
import subprocess

# Run libcamera-vid with HDR and save to a temporary file
subprocess.Popen("libcamera-vid --width 640 --height 480 --hdr -t 0 -o /dev/shm/live.h264", shell=True)

# Open video stream from file
cap = cv2.VideoCapture("/dev/shm/live.h264")

if not cap.isOpened():
    print("Error: Cannot access video stream!")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error reading frame!")
        break

    # Display image
    cv2.imshow("HDR Live Stream", frame)

    # Exit by pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release camera and close OpenCV windows
cap.release()
cv2.destroyAllWindows()
