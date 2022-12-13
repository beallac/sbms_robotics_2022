# LEGO type:standard slot:8 autostart

#importing all the functions
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import time
import math
hub = PrimeHub()
color_sensor = ColorSensor('A')
color_left = ColorSensor('B')
hub.light_matrix.show_image('SKULL')
wheels = MotorPair('F', 'C')
arm = Motor('D')
left_wheel = Motor('F')
right_wheel = Motor('C')


#this is defining a line following function
def Follow(power, whichColor, go_time, stripe_order):
    startTime = time.time()
    integral = 0
    lastError = 0
    while time.time()-startTime<go_time:
        error = whichColor.get_reflected_light() - 50
        P_fix = error * 0.3
        integral = integral + error # or integral+=error
        I_fix = integral * 0.001
        derivative = error - lastError
        lastError = error
        D_fix = derivative * 1
        correction = P_fix + I_fix + D_fix
        if stripe_order == "BW":
            correction = -correction

        wheels.start_tank_at_power(int(power+correction), int(power-correction))
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
    while -math.fabs(left_wheel.get_degrees_counted()) > 360*distance:
        error = hub.motion_sensor.get_yaw_angle() * 2
        wheels.start_tank(-speed-error, -speed+error-10)
        print('amogus', left_wheel.get_degrees_counted())
    wheels.stop()
#this is defining a line following function
def Follow_until_color(power, whichColor, whichColor2, stop_color, stripe_order):
    startTime = time.time()
    integral = 0
    lastError = 0
    while whichColor2.get_color() != stop_color:
        print(whichColor2.get_color())
        error = whichColor.get_reflected_light() - 50
        P_fix = error * 0.3
        integral = integral + error # or integral+=error
        I_fix = integral * 0.001
        derivative = error - lastError
        lastError = error
        D_fix = derivative * 1
        correction = P_fix + I_fix + D_fix
        if stripe_order == "BW":
            correction = -correction

        wheels.start_tank_at_power(int(power+correction), int(power-correction))
    wheels.stop


# wheels.move(4.4, unit='rotations', steering=2, speed=30)
go_straight(4.5, 45)
wheels.move(-.05, unit='rotations', steering=0, speed=30)
arm.run_for_degrees(-160, 30)
# wheels.move(-5.7, unit='rotations', steering=-7, speed=30)
go_straight_back(-5.7, 45)