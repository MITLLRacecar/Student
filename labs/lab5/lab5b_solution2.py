"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 5B - LIDAR Wall Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import enum

sys.path.insert(0, "../../library")
from racecar_core import rc
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################


class Mode(enum.IntEnum):
    align = 0
    right_panic = 1
    left_panic = 2


# >> Constants
# The maximum speed the car will travel
MAX_SPEED = 0.8

# When an object in front of the car is closer than this (in cm), start braking
BRAKE_DISTANCE = 150

# When a wall is within this distance (in cm), focus solely on not hitting that wall
PANIC_DISTANCE = 30

# When a wall is greater than this distance (in cm) away, exit panic mode
END_PANIC_DISTANCE = 32

# Speed to travel in panic mode
PANIC_SPEED = 0.3

# The minimum and maximum angles to consider when measuring closest side distance
MIN_SIDE_ANGLE = 10
MAX_SIDE_ANGLE = 60

# The angles of the two distance measurements used to estimate the angle of the left
# and right walls
SIDE_FRONT_ANGLE = 70
SIDE_BACK_ANGLE = 110

# When the front and back distance measurements of a wall differ by more than this
# amount (in cm), assume that the hallway turns and begin turning
TURN_THRESHOLD = 20

# The angle of measurements to average when taking an average distance measurement
WINDOW_ANGLE = 12

# >> Variables
cur_mode = Mode.align

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

    # Start in align mode
    cur_mode = Mode.align

    # Print start message
    print(">> Lab 5B - LIDAR Wall Following")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global cur_mode

    scan = rc.lidar.get_samples()
    speed = 0
    angle = 0

    # Find the minimum distance to the front, side, and rear of the car
    front_angle, front_dist = rc_utils.get_lidar_closest_point(
        scan, (-MIN_SIDE_ANGLE, MIN_SIDE_ANGLE)
    )
    left_angle, left_dist = rc_utils.get_lidar_closest_point(
        scan, (-MAX_SIDE_ANGLE, -MIN_SIDE_ANGLE)
    )
    right_angle, right_dist = rc_utils.get_lidar_closest_point(
        scan, (MIN_SIDE_ANGLE, MAX_SIDE_ANGLE)
    )

    # Estimate the left wall angle relative to the car by comparing the distance
    # to the left-front and left-back
    left_front_dist = rc_utils.get_lidar_average_distance(
        scan, -SIDE_FRONT_ANGLE, WINDOW_ANGLE
    )
    left_back_dist = rc_utils.get_lidar_average_distance(
        scan, -SIDE_BACK_ANGLE, WINDOW_ANGLE
    )
    left_dif = left_front_dist - left_back_dist

    # Use the same process for the right wall angle
    right_front_dist = rc_utils.get_lidar_average_distance(
        scan, SIDE_FRONT_ANGLE, WINDOW_ANGLE
    )
    right_back_dist = rc_utils.get_lidar_average_distance(
        scan, SIDE_BACK_ANGLE, WINDOW_ANGLE
    )
    right_dif = right_front_dist - right_back_dist

    # If we are within PANIC_DISTANCE of either wall, enter panic mode
    if left_dist < PANIC_DISTANCE or right_dist < PANIC_DISTANCE:
        cur_mode = Mode.left_panic if left_dist < right_dist else Mode.right_panic

    # If there are no visible walls to follow, stop the car
    if left_front_dist == 0.0 and right_front_dist == 0.0:
        speed = 0
        angle = 0

    # LEFT PANIC: We are close to hitting a wall to the left, so turn hard right
    elif cur_mode == Mode.left_panic:
        angle = 1
        speed = PANIC_SPEED

        if left_dist > END_PANIC_DISTANCE:
            cur_mode = Mode.align

    # RIGHT PANIC: We are close to hitting a wall to the right, so turn hard left
    elif cur_mode == Mode.right_panic:
        angle = -1
        speed = PANIC_SPEED

        if right_dist > END_PANIC_DISTANCE:
            cur_mode = Mode.align

    # ALIGN: Try to align straight and equidistant between the left and right walls
    else:
        # If left_dif is very large, the hallway likely turns to the left
        if left_dif > TURN_THRESHOLD:
            angle = -1

        # Similarly, if right_dif is very large, the hallway likely turns to the right
        elif right_dif > TURN_THRESHOLD:
            angle = 1

        # Otherwise, determine angle by taking into account both the relative angles and
        # distances of the left and right walls
        value = (right_dif - left_dif) + (right_dist - left_dist)
        angle = rc_utils.remap_range(
            value, -TURN_THRESHOLD, TURN_THRESHOLD, -1, 1, True
        )

        # Choose speed based on the distance of the object in front of the car
        speed = rc_utils.remap_range(front_dist, 0, BRAKE_DISTANCE, 0, MAX_SPEED, True)

    rc.drive.set_speed_angle(speed, angle)

    # Show the lidar scan, highlighting the samples used as min_dist measurements
    highlighted_samples = [
        (front_angle, front_dist),
        (left_angle, left_dist),
        (right_angle, right_dist),
    ]
    rc.display.show_lidar(scan, highlighted_samples=highlighted_samples)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print calculated statistics when the B button is held down
    if rc.controller.is_down(rc.controller.Button.B):
        print(
            "front_dist {:.2f}, left_dist {:.2f} cm, left_dif {:.2f} cm, right_dist {:.2f} cm, right_dif {:.2f} cm".format(
                front_dist, left_dist, left_dif, right_dist, right_dif
            )
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
