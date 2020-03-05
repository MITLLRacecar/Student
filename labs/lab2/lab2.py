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
CROP_BOTTOM = ((400, 0), (rc.camera.get_height(), rc.camera.get_width()))

# Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 50, 50), (110, 255, 255))  # The HSV range for the color blue
# TODO: add HSV ranges for other colors


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
        ">> Lab 2 - Image processing\n"
        "\n"
        "Controlls:\n"
        "   Right trigger = control forward speed\n"
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
    center = None
    area = 0

    # TODO: Implement a way to cycle through following multiple colors of tape

    if image is None:
        # If no image is found, center the wheels
        angle = 0
        center = None
        area = 0
    else:
        # Crop the image to only show the floor in front of the car
        cropped_image = rc_utils.crop(image, CROP_BOTTOM[0], CROP_BOTTOM[1])

        # Find all of the blue contours
        blueContours = rc_utils.find_contours(cropped_image, BLUE[0], BLUE[1])

        # Find the center and area of the largest blue contour
        largestBlueContour = rc_utils.get_largest_contour(
            blueContours, MIN_CONTOUR_SIZE
        )
        center = rc_utils.get_center(largestBlueContour)
        area = rc_utils.get_area(largestBlueContour)

    # Choose an angle based on center
    # If we could not find a contour center, keep the previous angle
    if center is not None:
        # TODO: Implement a smoother way for the car to follow the line
        if center[1] < rc.camera.get_width() / 2:
            angle = 1
        else:
            angle = -1

    # Use the right trigger to control the car's forward speed
    speed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if center is None:
            print("No contour found")
        else:
            print("Center:", center, "Area:", area)


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

    image = rc.camera.get_image()
    if image is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # Crop the image to only show the floor in front of the car
        cropped_image = rc_utils.crop(image, CROP_BOTTOM[0], CROP_BOTTOM[1])

        # Find all of the blue contours
        blueContours = rc_utils.find_contours(cropped_image, BLUE[0], BLUE[1])

        # Find the center and area of the largest blue contour
        largestBlueContour = rc_utils.get_largest_contour(
            blueContours, MIN_CONTOUR_SIZE
        )
        center = rc_utils.get_center(largestBlueContour)
        area = rc_utils.get_area(largestBlueContour)

        # If an image is found but no contour is found, print all dashes
        if center is None:
            print("-" * 32 + " : area = " + str(area))

        # Otherwise, print a line of dashes with a | where the line is seen
        else:
            s = ["-"] * 32
            s[int(center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(area))

            # Draw the contour onto the image
            cropped_image = rc_utils.draw_contour(
                cropped_image, largestBlueContour
            )

        # Display the image to the screen
        rc.display.show_image(cropped_image)


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
