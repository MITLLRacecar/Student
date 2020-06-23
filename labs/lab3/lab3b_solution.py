"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3B - Depth Camera Cone Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import enum

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

class Mode(enum.IntEnum):
    park = 0
    forward = 1
    reverse = 2


# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# Distance (in cm) we will park away from the cone
GOAL_DISTANCE = 30

# Distance (in cm) at which we switch from reverse to forward mode
FORWARD_DISTANCE = 120

# Distance (in cm) at which we switch from forward to reverse mode
REVERSE_DISTANCE = 60

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
cur_mode = Mode.forward

########################################################################################
# Functions
########################################################################################


def update_contour():
    """
    Finds the largest orange contour and use it to update contour_center
    """
    global contour_center

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
    else:
        # Find all of the orange contours
        contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None

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

    # Begin in "forward" mode
    cur_mode = Mode.forward

    # Print start message
    print(">> Lab 3B - Depth Camera Cone Parking")


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

    # Find the distance of the cone contour
    if contour_center is not None:
        depth_image = rc.camera.get_depth_image()
        distance = rc_utils.get_pixel_average_distance(depth_image, contour_center)

    # If no cone is found, stop
    if contour_center is None or distance == 0.0:
        speed = 0
        angle = 0

    else:
        # Use proportional control to set wheel angle based on contour x position
        angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)

        # PARK MODE: Move forward or backward until contour_area is GOAL_DISTANCE
        if cur_mode == Mode.park:
            speed = rc_utils.remap_range(
                distance, GOAL_DISTANCE * 2, GOAL_DISTANCE, 1.0, 0.0
            )
            speed = rc_utils.clamp(speed, -PARK_SPEED, PARK_SPEED)

            # If speed is close to 0, round to 0 to "park" the car
            if -SPEED_THRESHOLD < speed < SPEED_THRESHOLD:
                speed = 0

            # If the angle is no longer correct, choose mode based on area
            if abs(angle) > ANGLE_THRESHOLD:
                cur_mode = Mode.forward if distance > FORWARD_DISTANCE else Mode.reverse

        # FORWARD MODE: Move forward until we are closer that REVERSE_DISTANCE
        elif cur_mode == Mode.forward:
            speed = rc_utils.remap_range(
                distance, FORWARD_DISTANCE, REVERSE_DISTANCE, 1.0, 0.0
            )
            speed = rc_utils.clamp(speed, 0, ALIGN_SPEED)

            # Once we pass REVERSE_DISTANCE, switch to reverse mode
            if distance < REVERSE_DISTANCE:
                cur_mode = Mode.reverse

            # If we are close to the correct angle, switch to park mode
            if abs(angle) < ANGLE_THRESHOLD:
                cur_mode = Mode.park

        # REVERSE MODE: move backward until we are farther than FORWARD_DISTANCE
        else:
            speed = rc_utils.remap_range(
                distance, REVERSE_DISTANCE, FORWARD_DISTANCE, -1.0, 0.0
            )
            speed = rc_utils.clamp(speed, -ALIGN_SPEED, 0)

            # Once we pass FORWARD_DISTANCE, switch to forward mode
            if distance > FORWARD_DISTANCE:
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

    # Print the center and distance of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Distance:", distance)

    # Print the current mode when the X button is held down
    if rc.controller.is_down(rc.controller.Button.X):
        print("Mode:", cur_mode)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
