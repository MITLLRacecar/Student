"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 4A - IMU: Rolling Prevention
"""

################################################################################
# Imports
################################################################################

import sys

sys.path.insert(0, "../../library")
from racecar_core import *
import racecar_utils as rc_utils

import cv2 as cv
import numpy as np
import math


################################################################################
# Global variables
################################################################################

rc = racecar_core.create_racecar()

################################################################################
# Functions
################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """

    # Begin at a full stop
    rc.drive.stop()

    # Print start message
    print(
        ">> Lab 4A - IMU: Rolling Prevention \n"
        "\n"
        "Controlls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global queue
    global old_linear_velocity
    global current_linear_velocity

    # Calculate speed from triggers
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = forwardSpeed - backSpeed

    # Calculate angle from left joystick
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    # TODO (warmup): Prevent the car from turning too abruptly
    # Cap the angle based on the speed
    angle_cap = 1 - 0.25 * abs(speed)
    if angle > angle_cap:
        angle = angle_cap

    # Students may choose to adjust the angle, the speed, or both.
    # The example provides an upper limit for the angle based on the speed,
    # but you can also choose to scale the angle or speed.

    # Here is a list of other types of equations that the students may use.
    # Instances of angle and speed can be switched to adjust speed instead.
    # Formulas will have alpha, a user-defined constant.

    # # Linear scaling
    # alpha = 0.5
    # scaling_factor = 1 - alpha*abs(speed)
    # angle = scaling_factor*angle

    ## Reciprocal scaling
    # alpha = 10
    # angle_scaling = 1/(alpha*abs(speed) + 1)
    # angle = angle_scaling*angle

    ## Exponential scaling
    # alpha = 1.7
    # angle_scaling = -((math.e)**(abs(speed)/alpha))+2
    # angle = angle_scaling*angle

    # Set the final speed and angle
    rc.drive.set_speed_angle(speed, angle)


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
