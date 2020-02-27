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
import cv2 as cv
import numpy as np

################################################################################
# Global variables
################################################################################

rc = Racecar()
MIN_CONTOUR_SIZE = 30

SPEED = 0.0
ANGLE = 0
BLUE = ((0,0,0),(0,0,0))
SCREEN_CENTER = 320

################################################################################
# Functions
################################################################################

def crop(image, top_left, bottom_right):
    '''
    image: an image (these are stored as arrays, top left is (0,0))
    top_left: a pair of numbers representing the top left coordinate
    bottom_right: a pair of numbers representing the bottom right coordinate

    Helper function to make cropping images easier

    returns: a cropped version of the image
    '''
    x1, y1 = top_left
    x2, y2 = bottom_right
    return image[x1:x2,y1:y2]


def find_contours(img, HSV_lower, HSV_upper):
    """
    This function finds all contours in the image of the color range specified
    HSV min and max
    """
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, HSV_lower, HSV_upper)
    _,contours,_ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    return contours


def contours_exist(contours):
    """
    This function extracts the largest contour if it exists.
    If it doesn't exist, or is too small, it returns false.
    Else, it returns true along with the associated contour.
    """
    if len(contours) == 0:
        return (False, None)
    greatest_contour = max(contours, key=cv.contourArea)
    if cv.contourArea(greatest_contour) > MIN_CONTOUR_SIZE:
        return (True, greatest_contour)
    return (False, greatest_contour)


def get_center(contour):
    """
    This function takes an image, finds its greatest contour
    given a color range, then finds it's center. If it can't
    find a contour of the specified color range it returns
    the old angle.
    """

    # We want to find the center of the contour, also known
    # as the 1st moment of the contour

    M = cv.moments(contour)
    if M['m00'] == 0: # No pixels in contour
        return ANGLE

    # Now we compute the center of the contour
    contour_center = M['m10']/M['m00']
    return contour_center


def start():
    """
    This function is run once every time the start button is pressed
    """
    print("Started")
    global SPEED
    global ANGLE
    global BLUE

    # In this starter code, we will only mask the blue tape color
    BLUE = ((90,50,50), (110,255,255))


    #TODO: Mask for another color of tape

    rc.drive.set_speed_angle(SPEED, ANGLE)


def update():
    '''
    After start() is run, this function is run every frame until the back button
    is pressed
    '''
    global SPEED
    global ANGLE
    global BLUE

    image = rc.camera.get_image()

    #TODO: Implement a way to cycle through following multiple colors of tape

    if image is None:
        print("No Image")
        return

    else:
        hsv_lower, hsv_upper = BLUE
        exists, contour = contours_exist(find_contours(crop(image, (400,0), (480,640)), hsv_lower, hsv_upper))

        if exists:
            #TODO: Implement a smoother way for the car to follow the lines

            contour_center = get_center(contour)

            if contour_center < SCREEN_CENTER:
                ANGLE = 1
            elif contour_center > SCREEN_CENTER:
                ANGLE = -1
            else:
                ANGLE = 0

   # Use Trigger to set speed for better control
    forward_speed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    back_speed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    SPEED = (forward_speed - back_speed) if (forward_speed <= 0 or back_speed <= 0) else 0

    rc.drive.set_speed_angle(SPEED, ANGLE)

    # Print the current speed and angle when A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:",SPEED,"Angle:",ANGLE)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    image = rc.camera.get_image()
    if image is None:
        print("X"*32)
    else:
        hsv_lower, hsv_upper = BLUE
        exists, contour = contours_exist(find_contours(crop(image, (400,0), (480,640)), hsv_lower, hsv_upper))

        if exists:
            contour_center = get_center(contour)
            s = ["-"]*32
            s[int(contour_center / 20)] = "|"
            print("".join(s))
        else:
            print("-"*32)

################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
