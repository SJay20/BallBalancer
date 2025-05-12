from adafruit_servokit import ServoKit
from time import sleep

kit = ServoKit(channels=16)

kit.servo[0].actuation_range = 270
kit.servo[0].set_pulse_width_range(500, 2500)

while(True):
    kit.servo[0].angle = 0
    print("0")
    sleep(2)
    kit.servo[0].angle = 90
    print("90")
    sleep(2)
    kit.servo[0].angle = 180
    print("180")
    sleep(2)
    kit.servo[0].angle = 270
    print("270")
    sleep(2)
