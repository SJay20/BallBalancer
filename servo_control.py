from invkinematics import ThreeRPSInvKinematicsSolver
from adafruit_servokit import ServoKit
from time import sleep

# CONSTANTS
A = 1
B = 2
C = 3
SERVO_HOME = 180 - 22.894004

kit = ServoKit(channels=16)

servoA = kit.servo[0]
servoB = kit.servo[1]
servoC = kit.servo[2]

servoA.actuation_range = 270
servoA.set_pulse_width_range(500, 2500)

servoB.actuation_range = 270
servoB.set_pulse_width_range(500, 2500)

servoC.actuation_range = 270
servoC.set_pulse_width_range(500, 2500)

servoA.angle = SERVO_HOME
servoB.angle = SERVO_HOME
servoC.angle = SERVO_HOME
sleep(2)

solver = ThreeRPSInvKinematicsSolver(8.5, 9.0, 4.75, 5.0)
thetaA = solver.theta(A, 8, 0.2, 0)
thetaB = solver.theta(B, 8, 0.2, 0)
thetaC = solver.theta(C, 8, 0.2, 0)
print("Theta A: " + str(thetaA))
print("Theta B: " + str(thetaB))
print("Theta C: " + str(thetaC))

servoA.angle = 180 - thetaA
servoB.angle = 180 - thetaB
servoC.angle = 180 - thetaC

