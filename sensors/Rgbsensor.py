import board
import adafruit_tcs34725
import time

class Rgbsensor:
    def __init__(self):
        self.i2c = board.I2C()
        self.sensor = adafruit_tcs34725.TCS34725(self.i2c)

    def is_green(self):
        red = self.sensor.color_rgb_bytes[0]
        green = self.sensor.color_rgb_bytes[1]
        blue = self.sensor.color_rgb_bytes[2]
        if green > 2 * red and green > 2 * blue:
            return True
        else:
            return False


if __name__ == '__main__':
    test = Rgbsensor()
    while True:
        print(test.is_green())
        time.sleep(0.5)
