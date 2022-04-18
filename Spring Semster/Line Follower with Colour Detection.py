
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
import time
import cv2
import picamera
import numpy as np
from configparser import MAX_INTERPOLATION_DEPTH
from readline import set_completer_delims
import threading
from time import sleep

# Initialize camera
camera = picamera.PiCamera()
camera.resolution = (192,108)
camera.framerate = 20
rawCapture = PiRGBArray(camera,size=(192,108))
time.sleep(0.1)

#constant declarations
TARGET = 95
KP = 0.1

KD = 0.1
SAMPLETIME = 0.05
e_prev_error=0

# setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
in1=17
in2=22
in3=23
in4=24
enA=19
enB=16
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(enA,GPIO.OUT)
GPIO.setup(enB,GPIO.OUT)
pA=GPIO.PWM(enA,1000)
pB=GPIO.PWM(enB,1000)
pA.start(35)
pB.start(35)

# Loop over all frames captured by camera indefinitely
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
    # Display camera input
    image = frame.array
    cv2.imshow('img',image)
    #Rotate Image
    imageFrame=cv2.flip(image,-1)
    imageFrameA=imageFrame.copy()
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
 

        # Create key to break for loop
    key = cv2.waitKey(1) & 0xFF
    
    # Set range for red color and
    # define mask
    red_lower = np.array([136, 87, 111], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Set range for green color and
    # define mask
    green_lower = np.array([80, 153, 90], np.uint8)
    green_upper = np.array([100, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
    # Set range for blue color and
    # define mask
    blue_lower = np.array([100, 80, 100], np.uint8)
    blue_upper = np.array([140, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

 # Set range for yellow color and
    # define mask
    yellow_lower = np.array([19, 100, 100], np.uint8)
    yellow_upper = np.array([32, 255, 255], np.uint8)
    yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")

    # For red color
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame,
                              mask=red_mask)

    # For green color
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                mask=green_mask)

    # For blue color
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame,
                               mask=blue_mask)
    # For yellow color
    yellow_mask = cv2.dilate(yellow_mask, kernal)
    res_yellow = cv2.bitwise_and(imageFrame, imageFrame, mask=yellow_mask)

    # Creating contour to track red color
        r_contours, hierarchy = cv2.findContours(red_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
        for pic, r_contour in enumerate(r_contours):
            area = cv2.contourArea(r_contour)
            if(area > 300):
                x, y, w, h = cv2.boundingRect(r_contour)
                imageFrame = cv2.rectangle(imageFrame, (x, y),
                                        (x + w, y + h),
                                        (0, 0, 255), 2)

                cv2.putText(imageFrame, "Red Colour", (x, y),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                                        (0, 0, 255)) 
     # Creating contour to track green color
    g_contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    
    for pic, g_contour in enumerate(g_contours):
        area = cv2.contourArea(g_contour)
        if(area > 200):
            x, y, w, h = cv2.boundingRect(g_contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 255, 0), 2)

            cv2.putText(imageFrame, "Green Colour", (x, y),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1.0, (0, 255, 0))

    # Creating contour to track blue color
    b_contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    
    for pic, b_contour in enumerate(b_contours):
        area = cv2.contourArea(b_contour)
        if(area > 300):
            x, y, w, h = cv2.boundingRect(b_contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)

            cv2.putText(imageFrame, "Blue Colour", (x, y),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1.0, (255, 0, 0))


     # Creating contour to track yellow color
    y_contours, hierarchy = cv2.findContours(yellow_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, y_contour in enumerate(y_contours):
        area = cv2.contourArea(y_contour)
        if(area > 300):
            x, y, w, h = cv2.boundingRect(y_contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 255, 255), 2)

            cv2.putText(imageFrame, "Yellow Colour", (x, y),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1.0, (0, 255, 255))
 
    if len(g_contours)>0 or len(y_contours)>0:
        if len(g_contours)>0:
            c=max(g_contours,key=cv2.contourArea)
        elif len(y_contours)>0:
            c=max(y_contours,key=cv2.contourArea)
        M = cv2.moments(c)

        # Find x-axis centroid using image moments
        cx = int(M['m10']/M['m00'])
        print(cx)

        # Finding error
        e_error = 85 - cx
        deriative = e_error - e_prev_error

        # Calculation wheel Speeds
        speedL = 21 - (e_error * KP + deriative * KD)
        speedR = 21 + (e_error * KP + deriative * KD)
        print("cx {} error {}".format(cx, e_error))
        print("L {} R{}".format(speedL, speedR))

        # Wheel direction
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        GPIO.output(in3, GPIO.HIGH)
        GPIO.output(in4, GPIO.LOW)
        # Changing speed

        if speedR<0:
            speedR=0
        if speedL<0:
            speedL=0

        pA.ChangeDutyCycle(speedR)
        pB.ChangeDutyCycle(speedL)

        sleep(SAMPLETIME)
        e_prev_error=e_error
        print("coloured")
    

    else:
        # convert to grayscale, gaussian blur, and threshold
        gray = cv2.cvtColor(imageFrameA, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        ret,thresh1 = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)
        # Erode to eliminate noise, Dilate to restore eroded parts of image
        mask = cv2.erode(thresh1, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find all contours in frame
        contours, hierarchy = cv2.findContours(mask.copy(),1,cv2.CHAIN_APPROX_NONE)

        # Find x-axis centroid of largest contour and cut power to appropriate motor
        # to recenter camera on centroid.
        # This control algorithm was written referencing guide:
        # Author: Einsteinium Studios
        # Availability: http://einsteiniumstudios.com/beaglebone-opencv-line-following-robot.html

        if len(contours) > 0:
            # Find largest contour area and image moments
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c)

            # Find x-axis centroid using image moments
            cx = int(M['m10']/M['m00'])
            print(cx)
            print("line following")
            # Finding error
            e_error = 95 - cx
            deriative = e_error - e_prev_error
         # Calculation wheel Speeds
            speedL = 20 - (e_error * KP + deriative * KD)
            speedR = 20+ (e_error * KP + deriative * KD)
            print("cx {} error {}".format(cx, e_error))
            print("L {} R{}".format(speedL, speedR))

            # Wheel direction
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
            GPIO.output(in3, GPIO.HIGH)
            GPIO.output(in4, GPIO.LOW)
            # Changing speed

            if speedR<0:
                speedR=0
            if speedL<0:
                speedL=0

            pA.ChangeDutyCycle(speedR)
            pB.ChangeDutyCycle(speedL)

            sleep(SAMPLETIME)
            e_prev_error=e_error

    if key == ord("q"):
            break
        
    cv2.imshow("rotated",imageFrame)
    rawCapture.truncate(0)

GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)

GPIO.cleanup()





