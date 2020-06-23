"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3C - Depth Camera Wall Parking
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
# Distance (in cm) we will park away from the wall
GOAL_DIST = 20

# Distance (in cm) at which we switch from reverse to forward mode
FORWARD_DIST = 100

# Distance (in cm) at which we switch from forward to reverse mode
REVERSE_DIST = 50

# When the distances to LEFT_POINT and RIGHT_POINT are different by more than this
# amount (in cm), turn fully
MAX_DIST_DIF = 30

# Speed to use in parking and aligning modes
PARK_SPEED = 0.3
ALIGN_SPEED = 0.7

# The point in the depth image to measure right_dist and left_dist
LEFT_POINT = (rc.camera.get_height() // 2, int(rc.camera.get_width() * 1 / 4))
RIGHT_POINT = (rc.camera.get_height() // 2, int(rc.camera.get_width() * 3 / 4))

# The Kernel size to use when measuring distance at these points
KERNEL_SIZE = 11

# If desired speed/angle is under these thresholds, they are considered "close enough"
SPEED_THRESHOLD = 0.05
ANGLE_THRESHOLD = 0.1

# >> Variables
cur_mode = Mode.forward

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global cur_mode

    # Have the car begin at a stop
    rc.drive.stop()

    # Begin in "forward" mode
    cur_mode = Mode.forward

    # Print start message
    print(">> Lab 3C - Depth Camera Wall Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global cur_mode

    # Measure distance at the left, right, and center of the image
    depth_image = rc.camera.get_depth_image()
    center_dist = rc_utils.get_depth_image_center_distance(depth_image)
    left_dist = rc_utils.get_pixel_average_distance(
        depth_image, LEFT_POINT, KERNEL_SIZE
    )
    right_dist = rc_utils.get_pixel_average_distance(
        depth_image, RIGHT_POINT, KERNEL_SIZE
    )

    # Use the difference between left_dist and right_dist to determine angle
    dist_dif = left_dist - right_dist
    angle = rc_utils.remap_range(dist_dif, -MAX_DIST_DIF, MAX_DIST_DIF, -1, 1, True)

    # PARK MODE: More forward or backward until center_dist is GOAL_DIST
    if cur_mode == Mode.park:
        speed = rc_utils.remap_range(center_dist, GOAL_DIST * 2, GOAL_DIST, 1.0, 0.0)
        speed = rc_utils.clamp(speed, -PARK_SPEED, PARK_SPEED)

        # If speed is close to 0, round to 0 to "park" the car
        if -SPEED_THRESHOLD < speed < SPEED_THRESHOLD:
            speed = 0

        # If the angle is no longer correct, choose mode based on area
        if abs(angle) > ANGLE_THRESHOLD:
            cur_mode = Mode.forward if center_dist > FORWARD_DIST else Mode.reverse

    # FORWARD MODE: Move forward until we are closer that REVERSE_DIST
    elif cur_mode == Mode.forward:
        speed = rc_utils.remap_range(center_dist, FORWARD_DIST, REVERSE_DIST, 1.0, 0.0)
        speed = rc_utils.clamp(speed, 0, ALIGN_SPEED)

        # Once we pass REVERSE_DIST, switch to reverse mode
        if center_dist < REVERSE_DIST:
            cur_mode = Mode.reverse

        # If we are close to the correct angle, switch to park mode
        if abs(angle) < ANGLE_THRESHOLD:
            cur_mode = Mode.park

    # REVERSE MODE: move backward until we are farther than FORWARD_DIST
    else:
        speed = rc_utils.remap_range(center_dist, REVERSE_DIST, FORWARD_DIST, -1.0, 0.0)
        speed = rc_utils.clamp(speed, -ALIGN_SPEED, 0)

        # Once we pass FORWARD_DIST, switch to forward mode
        if center_dist > FORWARD_DIST:
            cur_mode = Mode.forward

        # If we are close to the correct angle, switch to park mode
        if abs(angle) < ANGLE_THRESHOLD:
            cur_mode = Mode.park

    # Reverse the angle if we are driving backward
    if speed < 0:
        angle *= -1

    rc.drive.set_speed_angle(speed, angle)

    # Display the depth image, and show LEFT_POINT and RIGHT_POINT
    rc.display.show_depth_image(depth_image, points=[LEFT_POINT, RIGHT_POINT])

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print measured distances when the B button is held down
    if rc.controller.is_down(rc.controller.Button.B):
        print(
            "left_dist:",
            left_dist,
            "center_dist:",
            center_dist,
            "right_dist:",
            right_dist,
        )

    # Print the current mode when the X button is held down
    if rc.controller.is_down(rc.controller.Button.X):
        print("Mode:", cur_mode)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
