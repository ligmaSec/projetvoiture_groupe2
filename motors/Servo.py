import PCA9685 as servo
import time


class Servo:
    def __init__(self):
        self.leftPWM = 350 # maximum left position of the servo
        self.homePWM = 450 # home position (90 degrees)
        self.rightPWM = 550 # right
        self.offset = -110 # calibration
        self.pwm = servo.PWM()
        self.leftPWM += self.offset
        self.homePWM += self.offset
        self.rightPWM += self.offset

        self.pwm.frequency = 50
    #def setup(self,busnum=None):
    def my_map(self,x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
   
    def turn_left(self):
        self.pwm.write(0, 0, self.leftPWM)

    def turn_right(self):
        self.pwm.write(0, 0, self.rightPWM)

    def turn(self,angle):
        if(angle < 0):
            angle = 0
        elif(angle > 255):
            angle = 255
        angle = self.my_map(angle, 0 ,255, self.leftPWM, self.rightPWM)
        self.pwm.write(0,0, int(angle))

    def home(self):
        self.pwm.write(0,0, self.homePWM)

    def calibrate(self,x):
        self.pwm.write(0,0, 450+x)

    def test(self):
        while True:
            self.turn_left()
            time.sleep(1)
            self.home()
            time.sleep(1)
            self.turn_right()
            time.sleep(1)


if __name__ == '__main__':
    letest = Servo()
    letest.test()
        
        

