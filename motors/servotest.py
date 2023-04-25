from Servo import Servo
import time
test = Servo()

def test():
    while True:
        for i in range(0,255,1):
            test.turn(i)
            time.sleep(0.01)

        for i in range(255,0,-1):
            test.turn(i)
            time.sleep(0.01)

test()

