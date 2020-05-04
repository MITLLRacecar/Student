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

sys.path.insert(0, "../../library")
from racecar_core import *
import racecar_utils as rc_utils

rospy.init_node("racecar")


################################################################################
# Global variables
################################################################################

rc = Racecar()

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
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controlls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
        "   A button = drive in a circle\n"
        "   B button = drive in a square\n"
        "   X button = drive in a figure eight\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO (warmup): Implement acceleration and steering
    rc.drive.set_speed_angle(0, 0)

    if rc.controller.was_pressed(rc.controller.Button.A):
        print("Driving in a circle...")
        # TODO (main challenge): Drive in a circle

    # TODO (main challenge): Drive in a square when the B button is pressed

    # TODO (main challenge): Drive in a figure eight when the X button is pressed

    # TODO (main challenge): Drive in a shape of your choice when the Y button
    # is pressed


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
