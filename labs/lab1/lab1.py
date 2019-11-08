"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 1 - Driving and Controller
"""

################################################################################
# Imports
################################################################################

import sys
sys.path.insert(0, '../../library')
from racecar_core import *
rospy.init_node('racecar')


################################################################################
# Global variables
################################################################################

rc = Racecar()
counter = 0
drive_function = None

SPEED = 1
ANGLE = 20

################################################################################
# Functions
################################################################################

def start():
    """
    This function is run once every time the start button is pressed
    """
    pass

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global counter
    global drive_function
    counter += rc.get_delta_time()

    # When the A button is pressed, begin driving in a circle
    if rc.controller.was_pressed(rc.controller.Button.A):
        drive_function = drive_circle
        counter = 0

    # When the B button is pressed, begin driving in a square
    if rc.controller.was_pressed(rc.controller.Button.B):
        drive_function = drive_square
        counter = 0
    
    # When the X button is pressed, begin driving in a figure eight
    if rc.controller.was_pressed(rc.controller.Button.X):
        drive_function = drive_figure_eight
        counter = 0

    # Calculate speed from triggers
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    speed = (forwardSpeed - backSpeed) * SPEED

    # If both triggers are pressed, stop for safety
    if (forwardSpeed > 0 and backSpeed > 0):
        speed = 0

    # Calculate angle from left joystick
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0] \
        * ANGLE

    # If the triggers are pressed, drive based on triggers + joystick
    if speed > 0:
        rc.drive(speed, angle)

    # Otherwise, drive based on the current drive function
    elif drive_function is not None:
        drive_function()
    
    else:
        rc.stop


def drive_circle(counter):
    CIRCLE_TIME = 2

    if counter < CIRCLE_TIME:
        rc.drive.set_speed_angle(SPEED, ANGLE)
    else:
        rc.drive.stop()


def drive_square(counter):
    STRAIGHT_TIME = 1
    TURN_TIME = 0.5

    for i in range(1, 5):
        if counter < STRAIGHT_TIME * i + TURN_TIME * (i - 1):
            rc.drive.set_speed_angle(SPEED, ANGLE)
            break
        if counter < STRAIGHT_TIME * i + TURN_TIME * i:
            rc.drive.set_speed_angle(SPEED, 0)
            break
    else:
        rc.drive.stop()


def drive_figure_eight(counter):
    STRAIGHT_TIME = 1
    TURN_TIME = 1.5

    for i in range(1, 3):
        if counter < STRAIGHT_TIME * i + TURN_TIME * (i - 1):
            rc.drive.set_speed_angle(SPEED, ANGLE)
            break
        if counter < STRAIGHT_TIME * i + TURN_TIME * i:
            rc.drive.set_speed_angle(SPEED, 0)
            break
    else:
        rc.drive.stop()
    
    
################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
