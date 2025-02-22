import cv2
import subprocess

# اجرای libcamera-vid برای دریافت جریان ویدیو با HDR فعال
command = [
    "libcamera-vid",
    "--width", "640",
    "--height", "480",
    "--hdr",  # فعال کردن HDR
    "-t", "0",  # زمان نامحدود
    "--codec", "mjpeg",
    "-o", "-"  # ارسال خروجی به stdout
]

# باز کردن استریم ویدیو از libcamera-vid
process = subprocess.Popen(command, stdout=subprocess.PIPE, bufsize=10**8)

# خواندن فریم‌ها از استریم
cap = cv2.VideoCapture("pipe:0", cv2.CAP_FFMPEG)

if not cap.isOpened():
    print("خطا: دریافت جریان ویدیو ممکن نیست!")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("خطا در دریافت فریم!")
        break

    cv2.imshow("HDR Live Stream", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
process.terminate()

