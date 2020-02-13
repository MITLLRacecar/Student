"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 2 - Line Following
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


################################################################################
# Functions
################################################################################

def crop(image, top_left, bottom_right):
    '''
    Helper function to make cropping images easier
    '''
    pass

def get_mask(image, hsv_low, hsv_high):
    '''
    Helper function to get the image mask given an HSV range
    '''
    pass

def get_contour(mask):
    '''
    Get a contour from the mask we created
    '''
    pass

def find_center(contour):
    '''
    Compute the coordinates of the center of our chosen contour
    '''
    pass

def start():
    '''
    This function is run once every time the start button is pressed
    '''
    pass

def update():
    '''
    After start() is run, this function is run every frame until the back button
    is pressed
    '''
    pass


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
