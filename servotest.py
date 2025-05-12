from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo
from time import sleep

servo = AngularServo(18, min_angle=0, max_angle=270, min_pulse_width=0.0004, max_pulse_width=0.0025, pin_factory=PiGPIOFactory())

while(True):
    servo.angle = 0
    sleep(2)
    servo.angle = 90
    sleep(2)
    servo.angle = 180
    sleep(2)
    servo.angle = 270
    sleep(2)