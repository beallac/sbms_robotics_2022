# LEGO type:standard slot:7 autostart

# importing all the functions
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import time
import math
# defining all the input ports
hub = PrimeHub()
gyro = hub.motion_sensor
color_sensor = ColorSensor('A')
color_left = ColorSensor('B')
hub.light_matrix.show_image('SKULL')
wheels = MotorPair('F', 'C')
arm = Motor('D')
left_wheel = Motor('F')
right_wheel = Motor('C')


def gyro_turn(direction, amount):
    if direction == 'right':
        while gyro.get_yaw_angle() < amount:
            wheels.start_tank(35, -35)
        wheels.stop()
    else:
        while gyro.get_yaw_angle() < amount:
            wheels.start_tank(-35, 35)
        wheels.stop()


def go_straight(distance, speed):
    hub.motion_sensor.reset_yaw_angle()
    left_wheel.set_degrees_counted(0)
    right_wheel.set_degrees_counted(0)
    while math.fabs(left_wheel.get_degrees_counted()) < 360*distance:
        error = hub.motion_sensor.get_yaw_angle() * 2
        wheels.start_tank(speed-error, speed+error)
        print(left_wheel.get_degrees_counted())
    wheels.stop()


def go_straight_back(distance, speed):
    hub.motion_sensor.reset_yaw_angle()
    left_wheel.set_degrees_counted(0)
    right_wheel.set_degrees_counted(0)
    while math.fabs(left_wheel.get_degrees_counted()) < 360*distance:
        error = hub.motion_sensor.get_yaw_angle() * 2
        wheels.start_tank(-speed-error, -speed+error)
        print(left_wheel.get_degrees_counted())
    wheels.stop()


def slow_gyro_turn(direction, amount):
    if direction == 'right':
        while gyro.get_yaw_angle() < amount:
            wheels.start_tank(25, -25)
        wheels.stop()
    else:
        while not gyro.get_yaw_angle() == amount:
            wheels.start_tank(-25, 25)
        wheels.stop()
# this is defining a line following function


def Follow(power, whichColor, go_time, stripe_order):
    startTime = time.time()
    integral = 0
    lastError = 0
    while time.time()-startTime < go_time:
        error = whichColor.get_reflected_light() - 50
        P_fix = error * 0.3
        integral = integral + error  # or integral+=error
        I_fix = integral * 0.001
        derivative = error - lastError
        lastError = error
        D_fix = derivative * 1
        correction = P_fix + I_fix + D_fix
        if stripe_order == "BW":
            correction = -correction

        wheels.start_tank_at_power(
            int(power+correction), int(power-correction))

# this is defining a line following function


def Follow_until_color(power, whichColor, whichColor2, stop_color, stripe_order):
    startTime = time.time()
    integral = 0
    lastError = 0
    while whichColor2.get_color() != stop_color:
        print(whichColor2.get_color())
        error = whichColor.get_reflected_light() - 50
        P_fix = error * 0.3
        integral = integral + error  # or integral+=error
        I_fix = integral * 0.001
        derivative = error - lastError
        lastError = error
        D_fix = derivative * 1
        correction = P_fix + I_fix + D_fix
        if stripe_order == "BW":
            correction = -correction

        wheels.start_tank_at_power(
            int(power+correction), int(power-correction))
    wheels.stop


# start
arm.run_for_degrees(-90, 40)
# wheels.move(-0.5, 'rotations', 0, 40)
gyro.reset_yaw_angle()
wheels.move(0.38, 'rotations', 0, 40)
gyro_turn('right', 47)
wheels.move(1.75, 'rotations', 0, 40)
arm.run_for_degrees(95, 40)
wheels.move(1, 'rotations', 30, 40)
wheels.move(1.5, 'rotations', -30, 40)
