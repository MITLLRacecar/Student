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
BRAKE_DISTANCE = 120

PANIC_DISTANCE = 30

PANIC_SPEED = 0.3

TURN_THRESHOLD = 20

WINDOW_ANGLE = 10

# >> 

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()



    # Print start message
    print(">> Lab 5B - LIDAR Wall Following")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    scan = rc.lidar.get_samples()
    highlighted_samples = []  # Samples we will highlight in the LIDAR visualization

    _, front_dist = rc_utils.get_lidar_closest_point(scan, (-10, 10))
    _, left_dist = rc_utils.get_lidar_closest_point(scan, (-60, -10))
    _, right_dist = rc_utils.get_lidar_closest_point(scan, (10, 60))

    lf = rc_utils.get_lidar_average_distance(scan, -70, WINDOW_ANGLE)
    lb = rc_utils.get_lidar_average_distance(scan, -110, WINDOW_ANGLE)
    ldif = lf - lb

    rf = rc_utils.get_lidar_average_distance(scan, 70, WINDOW_ANGLE)
    rb = rc_utils.get_lidar_average_distance(scan, 110, WINDOW_ANGLE)
    rdif = rf - rb

    speed = 0
    angle = 0
    if left_dist < PANIC_DISTANCE:
        print("LEFT PANIC")
        angle = 1
        speed = PANIC_SPEED
    elif right_dist < PANIC_DISTANCE:
        print("RIGHT PANIC")
        angle = -1
        speed = PANIC_SPEED
    else:
        if ldif > TURN_THRESHOLD:
            angle = -1
            print("left threshold")
        elif rdif > TURN_THRESHOLD:
            angle = 1
            print("right threshold")
        angle = rc_utils.remap_range(
            rdif - ldif, -TURN_THRESHOLD, TURN_THRESHOLD, -1, 1, True
        )
        speed = rc_utils.remap_range(front_dist, 0, BRAKE_DISTANCE, 0, MAX_SPEED, True)

    rc.drive.set_speed_angle(speed, angle)
    rc.display.show_lidar(scan, highlighted_samples=highlighted_samples)
    # print(
    #     "left_dist {}, right_dist {}, ldif {}, rdif {}".format(
    #         left_dist, right_dist, ldif, rdif
    #     )
    # )


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
