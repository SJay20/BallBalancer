from adafruit_servokit import ServoKit
from time import sleep

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

while(True):
    servo1.angle = 0
    servo2.angle = 90
    servo2.angle = 180
    sleep(2)
    servo1.angle = 90
    servo2.angle = 180
    servo2.angle = 270
    sleep(2)
    servo1.angle = 180
    servo2.angle = 270
    servo2.angle = 0
    sleep(2)
    servo1.angle = 270
    servo2.angle = 0
    servo2.angle = 90
    sleep(2)
