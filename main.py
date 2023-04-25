from sensors.Ultrasonic import Ultrasonic
from sensors.Infrared import Infrared
from sensors.Rgbsensor import Rgbsensor
from motors.DCmotor import DCmotor
from motors.Servo import Servo
from threading import Thread
import RPi.GPIO as g
import time
lapCount = 3
class CarController:
    def __init__(self, left_sensor, right_sensor, front_sensor,ir_sensor, rgb_sensor, motor, servo):
        self.left_sensor = left_sensor
        self.right_sensor = right_sensor
        self.front_sensor = front_sensor
        self.ir_sensor = ir_sensor
        self.rgb_sensor = rgb_sensor
        self.motor = motor
        self.servo = servo
        self.speed = 35
        self.speed_error = 0
        self.finish_counter = 0
        self.started = False

    def run(self):
        while self.finish_counter < lapCount: # finish line detection not yet implemented
            left_dist = self.left_sensor.distance()
            right_dist = self.right_sensor.distance()
            front_dist = self.front_sensor.distance()
            
            left_error =  left_dist - 20 
            right_error = right_dist - 20

            if not self.started:
                is_green = self.rgb_sensor.is_green()
                if is_green:
                    print("Green light detected, starting the car")
                    self.started = True
                else:
                    print("Waiting for green light ...")
                    time.sleep(1)
                    continue

            if front_dist < 20: 
                self.kp = 5  # steering angle multiplicator when close to an turn
                target_speed = 25
            else:
                self.kp = 2 # steering angle multiplicator in normal situation
                target_speed = 35
            steering_angle = int(self.kp * (left_error - right_error))

            # Adjust the servo position based on the steering angle
            current_position = 127 + steering_angle
            current_position = max(0, min(255, current_position))
            self.servo.turn(current_position)


            self.speed_error = target_speed - self.speed
            self.speed += self.speed_error / 10  # Adjust speed gradually

            self.motor.setSpeed(int(self.speed))

    def avoid_obstacles(self):
        left_dist = self.left_sensor.distance()
        right_dist = self.right_sensor.distance()
        front_dist = self.front_sensor.distance()

        # Check if there's an obstacle in front
        if front_dist < 20:
            # Turn away from the obstacle
            if left_dist > right_dist:
                steering_angle = -30
            else:
                steering_angle = 30
        else:
            # Otherwise, steer towards the center of the track
            left_error = left_dist - 20
            right_error = right_dist - 20
            steering_angle = int(self.kp * (left_error - right_error))

        # Adjust the servo position based on the steering angle
        current_position = 127 + steering_angle
        current_position = max(0, min(255, current_position))
        self.servo.turn(current_position)

    def follow_wall(self):
        # Set up initial values
        target_distance = 15  # Desired distance from wall
        min_distance = 10  # Minimum distance to stay away from wall
        max_distance = 20  # Maximum distance to stay away from wall
        steer_gain = 0.2  # Steering gain
        speed_gain = 0.2  # Speed gain

        # Loop until the method is stopped
        while True:
            # Read sensor values
            left_dist = self.left_sensor.distance()
            right_dist = self.right_sensor.distance()

            # Calculate error
            error = target_distance - left_dist

            # Limit error to min and max distance
            if error > max_distance:
                error = max_distance
            elif error < min_distance:
                error = min_distance

            # Calculate steering angle
            steering_angle = error * steer_gain

            # Calculate speed
            speed = self.speed + (target_distance - error) * speed_gain

            # Apply steering and speed
            current_position = 127 + steering_angle
            current_position = max(0, min(255, current_position))
            self.servo.turn(current_position)
            self.motor.setSpeed(int(speed))

            # Sleep for a short time to avoid overwhelming the system
            time.sleep(0.01)

    def do_circle(self):
        # Set initial values for speed and steering angle
        speed = 50
        steering_angle = -50
        
        # Turn the steering wheel to start turning in a circle
        self.servo.turn(127 + steering_angle)
        
        # Move the car in a circle for 5 seconds
        for _ in range(50):
            self.motor.setSpeed(speed)
            time.sleep(0.1)
        
        # Stop the car and reset the steering wheel position
        self.motor.setSpeed(0)
        self.servo.turn(127)



g.cleanup() # Clean up GPIO pins

ultrasonD = Ultrasonic(9, 11)
ultrasonG = Ultrasonic(19, 26)
ultrasonF = Ultrasonic(5, 6)
ir_sensor = Infrared(20)
rgb_sensor = Rgbsensor()
motor = DCmotor()
servo = Servo()

controller = CarController(ultrasonG, ultrasonD, ultrasonF,ir_sensor, rgb_sensor, motor, servo) # Passed all four parameters

car_thread = Thread(target=controller.run)
try:
    car_thread.daemon = True
    car_thread.start()
    while True:
        time.sleep(10)
except:
    motor.setSpeed(0)


