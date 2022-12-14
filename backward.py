# LEGO type:standard slot:9 autostart

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
        while not gyro.get_yaw_angle() == amount:
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


def go_straight_abs(distance, speed, direction):
    # hub.motion_sensor.reset_yaw_angle()
    left_wheel.set_degrees_counted(0)
    right_wheel.set_degrees_counted(0)

    while math.fabs(left_wheel.get_degrees_counted()) < 360*distance:
        error = (hub.motion_sensor.get_yaw_angle() - direction) * 2
        wheels.start_tank(speed-error, speed+error)
        print(left_wheel.get_degrees_counted())

    wheels.stop()

# ================================================================


def go_straight_abs_PID(distance, speed, direction):
    left_wheel.set_degrees_counted(0)
    right_wheel.set_degrees_counted(0)

    integral = 0
    lastError = 0
    while math.fabs(left_wheel.get_degrees_counted()) < 360*distance:
        error = (hub.motion_sensor.get_yaw_angle() - direction) * 3
        P_fix = error * 0.3
        integral = integral + error  # or integral+=error
        I_fix = integral * 0.001
        derivative = error - lastError
        lastError = error
        D_fix = derivative * 1
        correction = -(P_fix + I_fix + D_fix)

        wheels.start_tank_at_power(
            int(speed+correction), int(speed-correction))

    wheels.stop()

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

# ===================================================================================
# ===================================================================================


gyro.reset_yaw_angle

# actual begin
wheels.move(0.3, unit='rotations', steering=0, speed=40)
# wheels.move(-5.2, unit='rotations', steering=-2, speed=30)
go_straight_back(5.3, 45)
wheels.start_at_power(30)
intensity = color_left.get_reflected_light()
while intensity > 22:
    intensity = color_left.get_reflected_light()
    print(intensity)
print('found black')
wheels.stop()

# 0.75
# 0.2
wheels.move(0.25, unit='rotations', steering=0, speed=30)
wheels.move_tank(0.05, unit='rotations', left_speed=30, right_speed=-30)

# gyro_turn('left', 90)
slow_gyro_turn('right', 87)

# do turn, back up, lower arm, go forward
wheels.move(-0.5, unit='rotations', steering=0, speed=30)
arm.run_for_degrees(-185, 40)

# wheels.move(0.5, unit='rotations', steering=0, speed=30)
go_straight_abs_PID(0.5, 30, 90)
arm.run_for_degrees(75, 40)
arm.run_for_degrees(-75, 40)
arm.run_for_degrees(75, 40)
arm.run_for_degrees(-75, 40)
arm.run_for_degrees(75, 40)
arm.run_for_degrees(-75, 40)
arm.run_for_degrees(75, 40)
arm.run_for_degrees(-75, 40)
wheels.move(-0.5, unit='rotations', steering=0, speed=30)
wheels.move_tank(-0.35, unit='rotations', left_speed=30, right_speed=-30)
wheels.move(4.5, unit='rotations', steering=0, speed=55)
