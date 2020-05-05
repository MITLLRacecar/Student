"""
Copyright Harvey Mudd College
MIT License
Spring 2020

A lab demonstrating the Lidar capabilities.
"""

################################################################################
# Imports
################################################################################

import sys

sys.path.insert(0, "../library")
from racecar_core import *

rospy.init_node("racecar")


################################################################################
# Global variables
################################################################################

rc = Racecar()


################################################################################
# Functions
################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    length = rc.lidar.get_length()
    print (length)

    ranges = rc.lidar.get_ranges()
    print (ranges)


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    pass


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":

    rc.set_start_update(start, update)
    print rc.lidar.get_length()
    print rc.lidar.get_ranges()
    rc.go()
