"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 2 - Image Processing
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

# Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_SIZE = 30
# A crop window for the floor directly in front of the car
CROP_FLOOR = ((400, 0), (rc.camera.get_height(), rc.camera.get_width()))

# Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour = None  # The current contour used for navigation
contour_image = None  # The image from which the contour was extracted
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 50, 50), (110, 255, 255))  # The HSV range for the color blue
# TODO (challenge 1): add HSV ranges for other colors


################################################################################
# Functions
################################################################################


def update_contour(image):
    """
    Identifies a contour in the provided image to update global variables.

    Args:
        image: (2D numpy array of pixels) The image in which to find contours.

    Note:
        Updates the global variables contour, contour_image, contour_center,
        and contour_area.
    """
    global contour
    global contour_image
    global contour_center
    global contour_area

    contour_image = image

    if image is None:
        contour = None
        contour_center = None
        contour_area = 0
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        # (currently we only search for blue)

        # Find all of the blue contours
        contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_SIZE)

        # Update contour variables
        contour_center = rc_utils.get_center(contour)
        contour_area = rc_utils.get_area(contour)


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
        ">> Lab 2 - Image processing\n"
        "\n"
        "Controlls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
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

    image = rc.camera.get_image()

    # Crop the image to the floor directly in front of the car
    cropped_image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

    # Search for contours in the cropped image
    update_contour(cropped_image)

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if contour_center is not None:
        # TODO (warmup): Implement a smoother way to follow the line
        if contour_center[1] < rc.camera.get_width() / 2:
            angle = 1
        else:
            angle = -1

    # Use the triggers to control the car's speed
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = forwardSpeed - backSpeed

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)

    # TODO (challenge 2): When the left bumper (LB) is pressed, drive up to the closest
    # cone and stop six inches in front of it.


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # To help debug, update_slow does the following:
    # 1. Prints a line of ascii text to the console denoting the area of the contour
    #    and where the car sees the line
    # 2. Shows the most recent image to the screen with the largest contour drawn
    #    on top in bright green

    if contour_image is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | where the line is seen
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))

            # Draw the contour onto the image
            drawn_image = rc_utils.draw_contour(contour_image, contour)

        # Display the image to the screen
        rc.display.show_image(drawn_image)


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
