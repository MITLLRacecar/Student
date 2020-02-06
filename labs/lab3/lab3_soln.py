"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 9 - AR Lab
"""

################################################################################
# Imports
################################################################################

import sys
sys.path.insert(0, '../../library')
from racecar_core import *
rospy.init_node('racecar')

import cv2 as cv
from cv2 import aruco

################################################################################
# Global variables
################################################################################

RC = Racecar()
SPEED = 5
ANGLE = 0
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_100)

################################################################################
# Functions
################################################################################

def start():
    """
    This function is run once every time the start button is pressed
    """
    global SPEED
    global ANGLE
    # We also start the car driving
    RC.Drive.set_speed_angle(SPEED, ANGLE)

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global SPEED
    global ANGLE
    
    RC.Drive.set_speed(SPEED, ANGLE)


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    RC.set_start_update(start, update)
