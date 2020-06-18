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

sys.path.insert(0, "../../library")
from racecar_core import rc
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

# >> Constants
MAX_SPEED = 0.5

GOAL_DIST = 30

# Add any global variables here

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

    # Choose speed based on forward distance
    front_dist = rc_utils.get_lidar_average_distance(scan, 0)
    speed = rc_utils.remap_range(front_dist, 0, 200, 0, MAX_SPEED, True)

    left_ahead = rc_utils.get_lidar_average_distance(scan, 360 - 75)
    right_ahead = rc_utils.get_lidar_average_distance(scan, 75)

    highlighted_samples = []

    ratio = left_ahead / right_ahead if left_ahead > 0 and right_ahead > 0 else 1.0
    if front_dist < 200 and ratio > 2:
        angle = -1
        print("HARD LEFT: ratio", ratio, "front dist", front_dist)

    else:
        min_angle, min_side_dist = rc_utils.get_lidar_closest_point(scan, (15, 60))
        angle = rc_utils.remap_range(min_side_dist, GOAL_DIST / 2, GOAL_DIST, -1, 0)
        angle = rc_utils.clamp(angle, -1, 1)
        print("min side:", min_side_dist, "front", front_dist)
        highlighted_samples = [(min_angle, min_side_dist)]

    rc.drive.set_speed_angle(speed, angle)
    rc.display.show_lidar(scan, highlighted_samples=highlighted_samples)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
