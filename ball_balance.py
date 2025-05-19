from invkinematics import ThreeRPSInvKinematicsSolver
from adafruit_servokit import ServoKit
from time import sleep, time
from math import *
import numpy as np
import cv2
from picamera2 import Picamera2

# CONSTANTS
A = 1
B = 2
C = 3
SERVO_HOME = 180 - 22.894004

# initialize inverse kinematics solver (d, e, f, g)
solver = ThreeRPSInvKinematicsSolver(8.5, 9.0, 4.75, 5.0) 

# initialize PCA9685 servo driver
kit = ServoKit(channels=16) 

# initialize camera
picam2 = Picamera2()

# servo motors
servoA = kit.servo[0]
servoB = kit.servo[1]
servoC = kit.servo[2]

# initialize servos
servoA.actuation_range = 270 
servoA.set_pulse_width_range(500, 2500)
servoAOffset = 0

servoB.actuation_range = 270
servoB.set_pulse_width_range(500, 2500)
servoBOffset = 0

servoC.actuation_range = 270
servoC.set_pulse_width_range(500, 2500)
servoCOffset = 0

# PID variables
kp = 0
ki = 0
kd = 0

error = [0, 0]
errorPrev = [0, 0]
integral = [0, 0]
derivative = [0, 0]
output = [0, 0]

timeI = 0

ballDetected = False
pos = [0, 0]
clamp = lambda n, minn, maxn: max(min(maxn, n), minn)

#zero all servos
servoA.angle = SERVO_HOME
servoB.angle = SERVO_HOME
servoC.angle = SERVO_HOME
sleep(2)

def moveTo(hz, nx, ny):
    # if the ball is detected
    if ballDetected:
        # calculate motor positions
        for i in range(3):
            pos[i] = round(solver.theta(i, hz, nx, ny))
    # if the ball hasn't been detected
    else:
        for i in range(3):
            pos[i] = round(solver.theta(i, hz, 0, 0))
    
    # set the angles of the servos
    servoA.angle = servoAOffset + pos[A]
    servoB.angle = servoBOffset + pos[B]
    servoC.angle = servoCOffset + pos[C]

def PID(x, y):
    point = [0, 0]
    if(ballDetected):
        ballDetected = True
        # calculate PID values
        for i in range(2):
            errorPrev[i] = error[i]
            error[i] = (i == 0) * (point[0] - x) + (i == 1) * (point[1] - y)
            integral[i] += error[i] + errorPrev[i]
            derivative[i] = error[i] - errorPrev[i]   
            if isnan(derivative[i]) or isinf(derivative[i]):
                derivative[i] = 0   
            output[i] = kp * error[i] + ki * integral[i] + kd * derivative[i]
            output[i] = clamp(output[i], -0.25, 0.25);    

        print("X: " + output[0])
        print("Y: " + output[1])
    else:
        # double check for ball
        sleep(0.01)
        point = []
        ballDetected = False
    
    # continues moving platform and waits until 20 millis has elapsed
    timeI = time()
    while(time() - timeI < 20):
        moveTo(6, output[0], output[1])
    

while True:
    PID(0, 0)
