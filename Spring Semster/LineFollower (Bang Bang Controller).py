
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
TARGET = 85
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
    rotated=cv2.flip(image,-1)

        # Create key to break for loop
    key = cv2.waitKey(1) & 0xFF

        # convert to grayscale, gaussian blur, and threshold
    gray = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)
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

        
        #Turn Left
        if cx >= 120:
                    print("Turning Right")
                    pA.ChangeDutyCycle(25)
                    pB.ChangeDutyCycle(40)
                    GPIO.output(in1,GPIO.HIGH)
                    GPIO.output(in2,GPIO.LOW)
                    GPIO.output(in3,GPIO.HIGH)
                    GPIO.output(in4,GPIO.LOW)



        if cx < 120 and cx > 80:
                    print("Moving Forward")
                    pA.ChangeDutyCycle(50)
                    pB.ChangeDutyCycle(50)
                    GPIO.output(in1,GPIO.LOW)
                    GPIO.output(in2,GPIO.HIGH)
                    GPIO.output(in3,GPIO.HIGH)
                    GPIO.output(in4,GPIO.LOW)

        if cx <= 80:
                    print("Turning Left")
                    pA.ChangeDutyCycle(40)
                    pB.ChangeDutyCycle(25)
                    GPIO.output(in1,GPIO.LOW)
                    GPIO.output(in2,GPIO.HIGH)
                    GPIO.output(in3,GPIO.LOW)
                    GPIO.output(in4,GPIO.HIGH)

    
        

    if key == ord("q"):
            break
    cv2.imshow("threshold",thresh1)
    cv2.imshow("rotated",rotated)
    rawCapture.truncate(0)

GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)

GPIO.cleanup()

