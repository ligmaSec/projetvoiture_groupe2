import RPi.GPIO as GPIO
import time
 
 
class Ultrasonic:
    def __init__(self,GPIO_ECHO,GPIO_TRIGGER):
        GPIO.setmode(GPIO.BCM)
        self.GPIO_ECHO = GPIO_ECHO
        self.GPIO_TRIGGER = GPIO_TRIGGER
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

    def distance(self) -> float:
        #send a short trigger pulse
        GPIO.output(self.GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)
        StartTime = time.time()
        StopTime = time.time()
        # save StartTime
        while GPIO.input(self.GPIO_ECHO) == 0:
            StartTime = time.time()
        # save time of arrival
        while GPIO.input(self.GPIO_ECHO) == 1:
            StopTime = time.time()
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2 
        return distance

    def test(self):
        while True:
            dist = self.distance()
            yield dist
            time.sleep(0.001)
 
if __name__ == '__main__':
    test = Ultrasonic(9,11)
    for i in test.test():
        print(i)

