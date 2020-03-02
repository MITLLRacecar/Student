"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 2 - Image Processing
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

# Constants
MIN_CONTOUR_SIZE = 30   # The smallest
SCREEN_CENTER = 320     # The center x coordinate of the camera image

# Variables
speed = 0.0     # The current speed of the car
angle = 0.0     # The current angle of the car's wheels

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90,50,50), (110,255,255))  # The HSV range for the color blue
# TODO: add HSV ranges for other colors


################################################################################
# Functions
################################################################################

def crop(image, top_left, bottom_right):
    '''
    Crops an image to a rectangle based on the specified pixel points

    Inputs:
        image (2D numpy array of tripples): The image to crop
        top_left ((int, int)): The (row, column) of the top left pixel of the
            crop rectangle
        bottom_right((int, int)): The (row, column) of the bottom right pixel
            of the crop rectangle

    Output (2D numpy array of tripples): a cropped version of the image
    '''
    # Extract minimum and maximum pixel rows and columns from the parameters
    r_min, c_min = top_left
    r_max = bottom_right[0] + 1
    c_max = bottom_right[1] + 1

    # Shorten the array to the specified row and column ranges
    return image[r_min:r_max, c_min:c_max]


def find_contours(image, hsv_lower, hsv_upper):
    """
    Finds all contours of the specified color range in the provided image

    Inputs:
        image (2D numpy array of tripples): The image in which to find contours,
            with pixels represented in the bgr (blue-green-red) format
        hsv_lower ((int, int, int)): The lower bound for the hue, saturation,
            and value of colors to contour
        hsv_upper ((int, int, int)): The upper bound for the hue, saturation,
            and value of the colors to contour

    Output ([contours]): 
    """
    # Convert the image from a blue-green-red pixel representation to a
    # hue-saturation-value representation
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # Create a mask based on the pixels in the image with hsv values that
    # fall between HSV_lower and HSV_upper
    mask = cv.inRange(hsv_image, hsv_lower, hsv_upper)

    # Find and return a list of all contours of this mask
    _,contours,_ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    return contours


def contours_exist(contours):
    """
    Extracts the largest contour from a list of contours with an area
    greater than MIN_CONTOUR_SIZE

    Inputs:
        contours ([contours]): A list of contours found in an image

    Outputs:
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
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    # In this starter code, we will only mask the blue tape color
    BLUE = ((90,50,50), (110,255,255))

    #TODO: Mask for another color of tape

    rc.drive.set_speed_angle(speed, angle)


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
