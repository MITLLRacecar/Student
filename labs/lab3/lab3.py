"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 3 - Depth Camera
"""

################################################################################
# Imports
################################################################################

import sys

sys.path.insert(0, "../../library")
from racecar_core import *
import racecar_utils as rc_utils

rospy.init_node("racecar")
import cv2 as cv
import numpy as np


################################################################################
# Global variables
################################################################################

rc = Racecar()

# Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels

################################################################################
# Functions
################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab 3 - Depth Camera\n"
        "\n"
        "Controlls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle

    # Use the triggers to control the car's speed
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = forwardSpeed - backSpeed

    # Calculate the distance of the object directly in front of the car
    depth_image = rc.camera.get_depth_image()
    center_distance = rc_utils.get_center_distance(depth_image)

    # TODO (warmup): Prevent forward movement if the car is about to hit something

    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.was_pressed(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # TODO (challenge 1): When the left bumper (LB) is pressed, drive up to the closest
    # cone and stop six inches in front of it.  Your approach should use both color
    # and depth information and should work with cones of varying size.  You may
    # wish to reference lab 2.

    # TODO (challenge 2): When the right bumper (RB) is pressed, orient the car so that
    # it directly faces toward a wall (without hitting it).


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # To help debug, update_slow does the following:
    # 1. Prints a line of ascii text to the console denoting the area of the
    #    contour and where the car sees the line
    # 2. Shows the current image to the screen with the largest contour drawn
    #    on top in bright green

    depth_image = rc.camera.get_depth_image()

    if depth_image is None:
        print("No depth image found")
    else:
        # Calculate and print center depth
        center_depth = rc_utils.get_center_distance(depth_image)
        print("Depth at center: {} mm".format(center_depth))

        # Colorize and display the depth image to the screen
        colorized_depth = rc_utils.color_depth_image(depth_image)
        rc.display.show_image(colorized_depth)


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
