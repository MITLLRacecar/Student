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
# The maximum speed the car will travel
MAX_SPEED = 0.6

# When an object in front of the car is closer than this (in cm), start braking
BRAKE_DISTANCE = 150

# The window of angles the car will consider when measuring distance to the right wall
WINDOW_ANGLES = (15, 60)

# The distance (in cm) the car will attempt to stay from the right wall
GOAL_DIST = 30

# The angle (in degrees) that the car will look to its sides to decide when to turn left
SIDE_ANGLE = 75

# The amount the left distance must be larger than the right distance to turn left
LEFT_TURN_RATIO = 2

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

    # Choose speed based on forward distance
    _, front_dist = rc_utils.get_lidar_closest_point(scan, (-15, 15))
    speed = rc_utils.remap_range(front_dist, 0, BRAKE_DISTANCE, 0, MAX_SPEED, True)

    # Measure distance to the left and right walls ahead of the car
    left_ahead = rc_utils.get_lidar_average_distance(scan, -SIDE_ANGLE)
    right_ahead = rc_utils.get_lidar_average_distance(scan, SIDE_ANGLE)
    ratio = left_ahead / right_ahead if left_ahead > 0 and right_ahead > 0 else 1.0

    # If there is a wall ahead and the left wall is significantly father, assume that
    # the hallway has turned left
    if front_dist < BRAKE_DISTANCE and ratio > LEFT_TURN_RATIO:
        angle = -1
        print("HARD LEFT: ratio", ratio, "front dist", front_dist)

    # Otherwise, try to stay GOAL_DIST away from the right wall
    else:
        right_wall_angle, right_wall_dist = rc_utils.get_lidar_closest_point(
            scan, WINDOW_ANGLES
        )
        angle = rc_utils.remap_range(right_wall_dist, GOAL_DIST / 2, GOAL_DIST, -1, 0)
        angle = rc_utils.clamp(angle, -1, 1)

        # Display the closest point on the right wall
        highlighted_samples = [(right_wall_angle, right_wall_dist)]

    rc.drive.set_speed_angle(speed, angle)
    rc.display.show_lidar(scan, highlighted_samples=highlighted_samples)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
