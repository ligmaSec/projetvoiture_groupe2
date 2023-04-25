from motors.DCmotor import DCmotor
from motors.Servo import Servo
import RPi.GPIO as g
import time

moteur = DCmotor()
servo = Servo()
servo.home()
try:
    for i in range(10):
        moteur.setSpeed(50)
        moteur.forward()
        time.sleep(3)
        moteur.backwardWithSpeed()
        time.sleep(3)

    g.cleanup()
except KeyboardInterrupt:
    moteur.stop()
    servo.home()
    g.cleanup()
