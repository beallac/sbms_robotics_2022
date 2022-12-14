# LEGO type:standard slot:10 autostart

# importing all the functions
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import time
import math

# defining all the input ports
hub = PrimeHub()

color_sensor = ColorSensor('A')
color_left = ColorSensor('B')
hub.light_matrix.show_image('SKULL')
wheels = MotorPair('F', 'C')
arm = Motor('D')
left_wheel = Motor('F')
right_wheel = Motor('C')

gyro = hub.motion_sensor


def hand_turn(amo):
    while gyro.get_yaw_angle() < amo:
        wheels.start_tank(50, 0)

    wheels.stop

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
def go_straight_abs_PID(distance, speed, direction, maxTime):

    startTime = time.time()

    left_wheel.set_degrees_counted(0)
    right_wheel.set_degrees_counted(0)

    integral = 0
    lastError = 0

    while (math.fabs(left_wheel.get_degrees_counted()) < 360*distance) and (time.time()-startTime < maxTime):
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


def slow_gyro_turn(direction, amount):
    if direction == 'right':
        while gyro.get_yaw_angle() < amount:
            wheels.start_tank(25, -25)
        wheels.stop()
    else:
        while gyro.get_yaw_angle() > amount:
            wheels.start_tank(-35, 35)
        wheels.stop()


def gyro_turn(direction, amount):
    if direction == 'right':
        while gyro.get_yaw_angle() < amount:
            wheels.start_tank(35, -35)
        wheels.stop()
    else:
        while gyro.get_yaw_angle() > amount:
            wheels.start_tank(-35, 35)
        wheels.stop()


# ===================================================================================
# ===================================================================================

gyro.reset_yaw_angle()
# "azure","black","blue","cyan","green","orange","pink","red","violet","yellow","white"
hub.status_light.on('pink')
hub.speaker.beep(100, 0.1)
hub.motion_sensor.reset_yaw_angle()

while not (hub.left_button.is_pressed() or hub.right_button.is_pressed()):
    if hub.motion_sensor.get_yaw_angle() == 0:
        hub.status_light.on('green')
    else:
        hub.status_light.on('red')

# "azure","black","blue","cyan","green","orange","pink","red","violet","yellow","white"
hub.status_light.on('blue')
hub.speaker.beep(80, 0.1)

# ===================================================================================
# this is the TV challenge
# wheels.move(2.5, unit='rotations', steering=0, speed=55)
go_straight_abs_PID(2.5, 69, 0, 15)
wheels.move(-.85, unit='rotations', steering=0, speed=40)
gyro_turn('left', -36)


def yawspit():
    print('==============', gyro.get_yaw_angle())

# ===================================================================================
wheels.start(steering=0, speed=40)
# the robot is moving over to the wind turbine
color_sensor.wait_until_color('black')
wheels.stop()
wheels.move(0.45, unit='rotations', speed=40)
gyro_turn('right', 47)

# ===================================================================================
# now the robot is doing the wind turbine
wheels.move(2.3, unit='rotations', steering=0, speed=50)
wheels.move(-.5, unit='rotations', steering=0, speed=50)
wait_for_seconds(1)
wheels.move(1.8, unit='rotations', steering=0, speed=50)
wheels.move(-.5, unit='rotations', steering=0, speed=50)
wait_for_seconds(1)
wheels.move(1.8, unit='rotations', steering=0, speed=50)
wheels.move(-.5, unit='rotations', steering=0, speed=40)
wait_for_seconds(1)
wheels.move(1.8, unit='rotations', steering=0, speed=50)

# ===================================================================================
# now the robot is moving over to the hybrid car
wheels.move(-2.5, unit='rotations', steering=0, speed=50)
wheels.move(-0.3, unit='rotations', steering=0, speed=40)
wheels.move(0.05, unit='rotations', steering=0, speed=40)

# ===================================================================================
# slow_gyro_turn('left', -48.5)
gyro_turn('left', -48.5)
arm.run_for_degrees(-60)
wheels.move(1.5, unit='rotations', steering=0, speed=40)
wheels.start_at_power(25)
intensity = color_sensor.get_reflected_light()
while intensity < 90:
    intensity = color_sensor.get_reflected_light()
print('found white')
wheels.stop()
hub.speaker.beep(80, 0.1)
wheels.move(0.55, unit='rotations', steering=50, speed=40)

# ===================================================================================
# the robot is doing the hybrid car mission
gyro_turn('right', 33)
yawspit()

# ===================================================================================
# now the robot is moving over to the smart grid
# 0.5
wheels.move(-0.8, unit='rotations', steering=0, speed=40)
gyro_turn('left', -87.5)

yawspit()

yawspit()
wheels.move(0.4, 'rotations', 0, 40)
wheels.start_at_power(40)
intensity = color_left.get_reflected_light()
while intensity > 22:
    intensity = color_left.get_reflected_light()
    print(intensity)
    pass
print('found black')
wheels.stop()
# hand_turn(-1)
wheels.move_tank(0.6, 'rotations', 50, 0)
wheels.move(0.5, unit='rotations', steering=0, speed=40)

# ===================================================================================
# the robot is now doing the smart grid
arm.run_for_degrees(-110, 40)
wheels.move(-0.75, unit='rotations', steering=0, speed=25)
arm.run_for_seconds(1, 40)

# ===================================================================================
# The robot is sweeping up the energy units from the solar farm
# wheels.move_tank(0.95, unit = 'rotations', left_speed=0, right_speed=-85)
# 87
while gyro.get_yaw_angle() < 92.5:
    wheels.start_tank(left_speed=50, right_speed=-50)
wheels.stop()
wheels.move(-0.5, 'rotations', 0, 40)
while gyro.get_yaw_angle() < 165:
    wheels.start_tank(left_speed=0, right_speed=-50)
wheels.stop()
wheels.move(-1.6, unit='rotations', steering=0, speed=40)
wheels.move(0.05, unit='rotations', steering=0, speed=40)
wheels.move_tank(0.3, unit='rotations', left_speed=-40, right_speed=40)
wheels.move(-1.5, unit='rotations', steering=0, speed=40)
wheels.move_tank(-0.43, unit='rotations', left_speed=20, right_speed=-20)
# wheels.move_tank(0.75, unit='rotations', left_speed=-30, right_speed=-40)

# ===================================================================================
# The robot is returning home
wheels.move(-5, unit='rotations', steering=0, speed=50)


# ===================================================================================
# Play "Among Us" theme song
def note(note):
    hub.speaker.beep(note, 0.3)
    wait_for_seconds(0.1)


note(70)
wait_for_seconds(0.2)
note(70)
note(73)
note(75)
note(76)
note(75)
note(73)
note(70)
wait_for_seconds(0.4)
note(68)
note(73)
note(70)
wait_for_seconds(0.2)
note(68)
note(70)
