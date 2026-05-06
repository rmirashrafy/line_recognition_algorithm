import numpy as np
import time
import cv2

from picamera2 import Picamera2

# Grab images as numpy arrays and leave everything else to OpenCV.
cv2.startWindowThread()
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

# Function to get trackbar positions
def nothing(x):
    pass

cv2.namedWindow("Trackbars")
cv2.createTrackbar("L - H", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 200, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - S", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)
temp=0
count =0


while True:
    image= picam2.capture_array()
    if image is not None and image.any():
        frame = cv2.resize(image, (640, 480))

        # Perspective transformation points

        tl = (80, 100) 
        bl = (4, 373) 
        tr = (590, 100) 
        br = (634, 373)
        cv2.circle(frame, tl, 5, (0,0,255), -1)
        cv2.circle(frame, bl, 5, (0,0,255), -1)
        cv2.circle(frame, tr, 5, (0,0,255), -1)
        cv2.circle(frame, br, 5, (0,0,255), -1)
        pts1 = np.float32([tl, bl, tr, br]) 
        pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]]) 
        matrix = cv2.getPerspectiveTransform(pts1, pts2) 
        transformed_frame = cv2.warpPerspective(frame, matrix, (640, 480))

        # Convert image to HSV and apply thresholding
        hsv_transformed_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2HSV)
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
        lower, upper = np.array([l_h, l_s, l_v]), np.array([u_h, u_s, u_v])
        mask = cv2.inRange(hsv_transformed_frame, lower, upper)
        histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        #Sliding Window
        y = 472
        lx = []
        rx = []

        msk = mask.copy()

        while y>0:
            ## Left threshold
            img = mask[y-40:y, left_base-50:left_base+50]
            contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"]/M["m00"])
                    cy = int(M["m01"]/M["m00"])
                    lx.append(left_base-50 + cx)
                    left_base = left_base-50 + cx
            
            ## Right threshold
            img = mask[y-40:y, right_base-50:right_base+50]
            contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"]/M["m00"])
                    cy = int(M["m01"]/M["m00"])
                    lx.append(right_base-50 + cx)
                    right_base = right_base-50 + cx
            distance = abs(right_base - left_base)
            # if distance >515 and distance < 567 :
            #     distance = distance-515 #right
            #     car.setSteering(distance)
            # elif distance >= 567 and distance <620:
            #     distance = distance-535 #right
            #     car.setSteering(distance)
            # elif distance >= 620 and distance <640:
            #     distance = distance-610 #right
            #     car.setSteering(distance)
            # #################################
            # elif distance > 321 and distance <= 419:
            #     distance = (-distance) + 321#left
            #     car.setSteering(distance)
            # elif distance > 419 and distance < 498 :
            #     distance = (-distance)+397#left
            #     car.setSensorAngle(distance)
            # # elif distance >= 488 and distance < 498 :
            # #     distance = (-distance)+417#left
            # #     car.setSensorAngle(distance)
            # elif distance > 499 and distance < 514:
            #     car.setSteering(0)
            # elif distance == 320:
            #     car.setSteering(90)
            # elif distance == 0:
            #     car.setSteering(-90)

            print(f"Distance between lines at y={y}: {distance} pixels")
            count = count+1
            #print(count)
            if (count  > 17):
                count=0
                temp=0
            cv2.rectangle(msk, (left_base-50,y), (left_base+50,y-40), (255,255,255), 2)
            cv2.rectangle(msk, (right_base-50,y), (right_base+50,y-40), (255,255,255), 2)
            y -= 40

        # Display images
        cv2.imshow("Original", frame)
        cv2.imshow("Bird's Eye View", transformed_frame)
        cv2.imshow("Lane Detection - Thresholding", mask)
        cv2.imshow("Lane Detection - Sliding Windows", msk)

        if cv2.waitKey(10) == 27:
            break

