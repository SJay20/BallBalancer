from invkinematics import ThreeRPSInvKinematicsSolver
from adafruit_servokit import ServoKit
from time import sleep

# CONSTANTS
A = 1
B = 2
C = 3
SERVO_HOME = 180 - 22.894004

kit = ServoKit(channels=16)

servo1 = kit.servo[0]
servo2 = kit.servo[1]
servo3 = kit.servo[2]

servo1.actuation_range = 270
servo1.set_pulse_width_range(500, 2500)

servo2.actuation_range = 270
servo2.set_pulse_width_range(500, 2500)

servo3.actuation_range = 270
servo3.set_pulse_width_range(500, 2500)

servo1.angle = SERVO_HOME
servo2.angle = SERVO_HOME
servo3.angle = SERVO_HOME
sleep(2)

solver = ThreeRPSInvKinematicsSolver(8.5, 9.0, 4.75, 5.0)
thetaA = solver.theta(A, 8, 0.2, 0)
thetaB = solver.theta(B, 8, 0.2, 0)
thetaC = solver.theta(C, 8, 0.2, 0)
print("Theta A: " + str(thetaA))
print("Theta B: " + str(thetaB))
print("Theta C: " + str(thetaC))

servo1.angle = 180 - thetaA
servo2.angle = 180 - thetaB
servo3.angle = 180 - thetaC

