# LEGO type:standard slot:1 autostart

"""
πRATS 2022 - FLL TEAM # 34612 - SANTA BARBARA MIDDLE SCHOOL
Battery Bandit prototype
"""

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
hub = PrimeHub()
hub.light_matrix.show_image('ANGRY')

belt = Motor('B')
boxes = Motor('D')
color_sensor = ColorSensor('F')

dd = DistanceSensor('A')

dd.light_up_all()
while True:
    dd.wait_for_distance_closer_than(18.0, unit='cm')
    sensorDistance = dd.get_distance_cm()
    print(sensorDistance)
    hub.speaker.beep(100, 0.1)
    break 

belt.start(speed=-40)

while True:
    color = 'none'

    while color != 'green' and color != 'blue' and color != 'yellow':
        color = color_sensor.get_color()
    belt.stop()

    # Move bins for color
    if color == 'green':
        boxes.run_for_rotations(-2, speed=60)

    elif color == 'yellow':
        boxes.run_for_rotations(2, speed=60)

    elif color == 'blue':
        pass

    # Move belt to dump item
    belt.start(speed=-45)

    # Reset bins to center
    wait_for_seconds(2)
    belt.stop()
    if color == 'green':
        boxes.run_for_rotations(2, speed=60)

    elif color == 'yellow':
        boxes.run_for_rotations(-2, speed=60)

    elif color == 'blue':
        pass

    belt.start(speed=-45)
