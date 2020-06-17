"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2B - Color Image Cone Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import enum

sys.path.insert(1, "../../library")
from racecar_core import rc
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################


class Mode(enum.IntEnum):
    park = 0
    forward = 1
    reverse = 2


# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# Area of the cone contour when we are the correct distance away (must be tuned)
GOAL_AREA = 24000

# Area of the cone contour when we should switch to reverse while aligning
REVERSE_AREA = GOAL_AREA * 0.6

# Area of the cone contour when we should switch to forward while aligning
FORWARD_AREA = GOAL_AREA * 0.4

# Speed to use in parking and aligning modes
PARK_SPEED = 0.3
ALIGN_SPEED = 0.7

# The HSV range for the color orange, stored as (hsv_min, hsv_max)
ORANGE = ((10, 50, 50), (20, 255, 255))

# If desired speed/angle is under these thresholds, they are considered "close enough"
SPEED_THRESHOLD = 0.05
ANGLE_THRESHOLD = 0.1

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
cur_mode = Mode.forward

########################################################################################
# Functions
########################################################################################


def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Find all of the orange contours
        contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0

        # Display the image to the screen
        rc.display.show_color_image(image)


def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global cur_mode

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Begin in "forward" mode
    cur_mode = Mode.forward

    # Print start message
    print(">> Lab 2B - Color Image Cone Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global cur_mode

    # Search for contours in the current color image
    update_contour()

    # If no cone is found, stop
    if contour_center is None or contour_area == 0:
        speed = 0
        angle = 0

    else:
        # Use proportional control to set wheel angle based on contour x position
        angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)

        # PARK MODE: Move forward or backward until contour_area is GOAL_AREA
        if cur_mode == Mode.park:
            speed = rc_utils.remap_range(
                contour_area, GOAL_AREA / 2, GOAL_AREA, 1.0, 0.0
            )
            speed = rc_utils.clamp(-PARK_SPEED, PARK_SPEED, speed)

            # If speed is close to 0, round to 0 to "park" the car
            if -SPEED_THRESHOLD < speed < SPEED_THRESHOLD:
                speed = 0

            # If the angle is no longer correct, choose mode based on area
            if abs(angle) > ANGLE_THRESHOLD:
                cur_mode = Mode.forward if contour_area < FORWARD_AREA else Mode.reverse

        # FORWARD MODE: Move forward until area is greater than REVERSE_AREA
        elif cur_mode == Mode.forward:
            speed = rc_utils.remap_range(
                contour_area, MIN_CONTOUR_AREA, REVERSE_AREA, 1.0, 0.0
            )
            speed = rc_utils.clamp(0, ALIGN_SPEED, speed)

            # Once we pass REVERSE_AREA, switch to reverse mode
            if contour_area > REVERSE_AREA:
                cur_mode = Mode.reverse

            # If we are close to the correct angle, switch to park mode
            if abs(angle) < ANGLE_THRESHOLD:
                cur_mode = Mode.park

        # REVERSE MODE: move backward until area is less than FORWARD_AREA
        else:
            speed = rc_utils.remap_range(
                contour_area, REVERSE_AREA, FORWARD_AREA, -1.0, 0.0
            )
            speed = rc_utils.clamp(-ALIGN_SPEED, 0, speed)

            # Once we pass FORWARD_AREA, switch to forward mode
            if contour_area < FORWARD_AREA:
                cur_mode = Mode.forward

            # If we are close to the correct angle, switch to park mode
            if abs(angle) < ANGLE_THRESHOLD:
                cur_mode = Mode.park

        # Reverse the angle if we are driving backward
        if speed < 0:
            angle *= -1

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

    # Print the current mode when the X button is held down
    if rc.controller.is_down(rc.controller.Button.X):
        print("Mode:", cur_mode)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
