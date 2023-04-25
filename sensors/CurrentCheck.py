import adafruit_ina219
import board
import busio
from time import sleep

class CurrentCheck:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ina219 = adafruit_ina219.INA219(i2c)

    def current(self):
        courant = self.ina219.current
        return courant


if __name__ == '__main__':
    test = CurrentCheck()
    while True:
        print(f"{test.current()} mA")
        sleep(0.1)
