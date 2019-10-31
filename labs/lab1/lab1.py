"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 1 - Driving and Controller
"""

################################################################################
# Imports
################################################################################

import sys
sys.path.insert(0, '../../library')
from racecar_core import *
rospy.init_node('racecar')


################################################################################
# Global variables
################################################################################

rc = Racecar()
counter = 0


################################################################################
# Functions
################################################################################

def start():
    """
    This function is run once every time the start button is pressed
    """
    print("start")

def update():
    """
    After start() is run, this function is run every frame until the back button is
    pressed
    """
    global counter
    print(counter)
    # if (counter < 120):
    #     rc.drive.set_speed_angle(1, 20)
    # else:
    #     rc.drive.set_speed_angle(0, -10)
    counter += 1


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
