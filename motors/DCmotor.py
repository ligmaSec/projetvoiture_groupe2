#!/usr/bin/env python3
#coding: utf-8

import RPi.GPIO as g
import time
import PCA9685 as p
class DCmotor:
    def __init__(self):
        # ===========================================================================
        # Raspberry Pi pin11, 12, 13 and 15 to realize the clockwise/counterclockwise
        # rotation and forward and backward movements
        # ===========================================================================
        self.Motor0_A = 17  # pin11
        self.Motor0_B = 18  # pin12
        self.Motor1_A = 27  # pin13
        self.Motor1_B = 22  # pin15
        # ===========================================================================
        # Set channel 4 and 5 of the DC motor driver IC to generate PWM, thus 
        # controlling the speed of the car
        # ===========================================================================
        self.EN_M0    = 4  # servo driver IC CH4
        self.EN_M1    = 5  # servo driver IC CH5
        self.pins = [self.Motor0_A, self.Motor0_B, self.Motor1_A, self.Motor1_B]
        
        #test

        self.backward1 = 'False'
        self.backward0 = 'False'
        self.pwm = p.PWM()                  # Initialize the servo controller.

        self.pwm.frequency = 60
        self.forward0 = 'True'
        self.forward1 = 'True'
        g.setwarnings(False)
        g.setmode(g.BCM)        # Number g. by its physical location
        if self.forward0 == 'True':
            self.backward0 = 'False'
        elif self.forward0 == 'False':
            self.backward0 = 'True'
        if self.forward1 == 'True':
            self.backward1 = 'False'
        elif self.forward1 == 'False':
            self.backward1 = 'True'
        for pin in self.pins:
            g.setup(pin, g.OUT)

    def setSpeed(self, speed):
        speed *= 40
        #print ('speed is: ', speed)
        self.pwm.write(self.EN_M0, 0, speed)
        self.pwm.write(self.EN_M1, 0, speed)

        
    def motor0(self, x):
        if x == 'True':
            g.output(self.Motor0_A, g.LOW)
            g.output(self.Motor0_B, g.HIGH)
        elif x == 'False':
            g.output(self.Motor0_A, g.HIGH)
            g.output(self.Motor0_B, g.LOW)
        else:
            print('Config Error')

    def motor1(self, x):
        if x == 'True':
            g.output(self.Motor1_A, g.LOW)
            g.output(self.Motor1_B, g.HIGH)
        elif x == 'False':
            g.output(self.Motor1_A, g.HIGH)
            g.output(self.Motor1_B, g.LOW)

    def forward(self):
        self.motor0(self.forward0)
        self.motor1(self.forward1)

    def backward(self):
        self.motor0(self.backward0)
        self.motor1(self.backward1)

    def forwardWithSpeed(self, spd=50):
        self.setSpeed(spd)
        self.motor0(self.forward0)
        self.motor1(self.forward1)

    def backwardWithSpeed(self, spd=50):
        self.setSpeed(spd)
        self.motor0(self.backward0)
        self.motor1(self.backward1)

    def stop(self):
        for pin in self.pins:
            g.output(pin, g.LOW)

    def ctrl(self, status, direction=1):
        if status == 1:   # Run
            if direction == 1:     # Forward
                self.forward()
            elif direction == -1:  # Backward
                self.backward()
            else:
                print('Argument error! direction must be 1 or -1.')
        elif status == 0: # Stop
            self.stop()
        else:
            print('Argument error! status must be 0 or 1.')

    def test(self):
        while True:
            self.setup()
            self.ctrl(1)
            time.sleep(3)
            self.setSpeed(10)
            time.sleep(3)
            self.setSpeed(100)
            time.sleep(3)
            self.ctrl(0)

if __name__ == '__main__':
    test = DCmotor()
    test.test()
