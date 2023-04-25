import RPi.GPIO as GPIO
import time
#set en mode norme gpio != board
GPIO.setmode(GPIO.BCM)
class Infrared:
    def __init__(self,pin) :
        self.__pin = pin
        # Set Ir pin in "in" mode (Receiving) 
        GPIO.setup(pin, GPIO.IN)

    def read(self):
        # Return result (0: RGB, 1: black)
        return GPIO.input(self.__pin)


if __name__ == '__main__':

    test = Infrared(20)
    while True:
        print(test.read())
        time.sleep(0.1)
