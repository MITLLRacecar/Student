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
import racecar_core
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

    # Get angular velocity from IMU
    ang_vel = rc.physics.get_angular_velocity()

    # TODO (warmup): Prevent the car from turning too abruptly
    # Cap the angle if the car turns too quickly
    # Tune the constant to avoid flipping
    ANG_VEL_MAX = 0.50
    angle_cap = angle*(ANG_VEL_MAX/ang_vel[1])
    if ang_vel[1] > ANG_VEL_MAX:
        angle = angle_cap
    if ang_vel[1] < -ANG_VEL_MAX:
        angle = -angle_cap

    # Set the final speed and angle
    rc.drive.set_speed_angle(speed, angle)


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
