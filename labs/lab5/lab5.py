"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 5 - AR Markers
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

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
    print(">> Lab 5 - AR Markers")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)

    # TODO: At the first AR marker, turn left if the ID is 0 and right if the ID is 1

    # TODO: At the second AR marker, turn left if the marker faces left and right if
    # the marker faces right

    # TODO: At the third AR marker, follow the color line which matches the color border
    # surrounding the marker


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
