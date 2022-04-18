#TYhis program is to calculate the distance travelled by the robot car
import RPi.GPIO as GPIO
import time

#pin declaration
in1=17
in2=22
in3=23
in4=24
enA=19
enB=16
M=18
#M is the output pin for the rotary encoder
#SET PIN MODES
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(enA,GPIO.OUT)
GPIO.setup(enB,GPIO.OUT)
GPIO.setup(M,GPIO.IN)
pA=GPIO.PWM(enA,1000)
pB=GPIO.PWM(enB,1000)
pA.start(100)
pB.start(100)


GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.HIGH)
GPIO.output(in3,GPIO.HIGH)
GPIO.output(in4,GPIO.LOW)

stateLast=GPIO.input(M)
rotationCount=0
stateCount=0
stateCountTotal=0

try:
    while 1:
        stateCurrent=GPIO.input(M)
        if stateCurrent != stateLast:
            stateLast=stateCurrent
            stateCount+=1
            stateCountTotal+=1
    
        distance=((stateCountTotal*9.0)/360.0)*20.42
        print("Distance",distance)

#Press and key to stop the robot car movement
except KeyboardInterrupt:
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    GPIO.cleanup()

