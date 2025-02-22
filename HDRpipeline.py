import cv2
import subprocess

# اجرای libcamera-vid با HDR و ذخیره در یک فایل موقت
subprocess.Popen("libcamera-vid --width 640 --height 480 --hdr -t 0 -o /dev/shm/live.h264", shell=True)

# باز کردن جریان ویدیو از فایل
cap = cv2.VideoCapture("/dev/shm/live.h264")

if not cap.isOpened():
    print("خطا: دریافت جریان ویدیو ممکن نیست!")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("خطا در دریافت فریم!")
        break

    # نمایش تصویر
    cv2.imshow("HDR Live Stream", frame)

    # خروج با فشردن کلید 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# بستن دوربین و OpenCV
cap.release()
cv2.destroyAllWindows()

