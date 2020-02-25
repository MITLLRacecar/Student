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
import cv2
import numpy as np

################################################################################
# Global variables
################################################################################

RC = Racecar()
MIN_CONTOUR_SIZE = 30
LINE_COLOR_PRIORITY = list()
SPEED = 0.0
ANGLE = 0

################################################################################
# Functions
################################################################################

def crop(img, tl, br):
    """
    This function is used to crop our image to the desired box
    """
    
    x1, y1 = tl
    x2, y2 = br
    return img[x1:x2,y1:y2]

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

def get_angle(contour, curr_traj):
    """
    This function takes an image, finds its greatest contour
    given a color range, then finds it's center and computes
    the angle to turn the car to. If it can't find a contour
    of the specified color range it returns the old angle.
    """
    screen_center = 320

    speed, angle = curr_traj

    # We want to find the center of the contour, also known
    # as the 1st moment of the contour
    M = cv.moments(contour)
    if M['m00'] == 0: # No pixels in contour
        return angle

    # Now we compute the desired angle
    contour_center = M['m10']/M['m00']
    error = contour_center - screen_center
    ratio = error/screen_center
    max_angle = -1.0
    if speed < 0:
        max_angle = 1.0
    return ratio*max_angle

def start():
    """
    This function is run once every time the start button is pressed
    """
    print("Started")
    global SPEED
    global ANGLE
    
    # In this starter code, we will only mask teh blue tape color
    blue = ((90,50,50), (110,255,255))
    LINE_COLOR_PRIORITY.append(blue)

    #TODO: Mask for other colors of tape
    # then add their color bounds to the LINE_COLOR_PRIORITY 

    RC.drive.set_speed_angle(SPEED, ANGLE)
     

def update():
    '''
    After start() is run, this function is run every frame until the back button
    is pressed
    '''
    global SPEED
    global ANGLE
    image = RC.camera.get_image()

    if image is None:
        print("No Image")
        return
    
    for color_bound in LINE_COLOR_PRIORITY:
        # We get the upper and lower bounds for the color that we care about
        
        hsv_lower, hsv_upper = color_bound
        exists, contour = contours_exist(find_contours(crop(image, (400,0), (480,640)), hsv_lower, hsv_upper))
        
        if exists:
            ANGLE = get_angle(contour, (SPEED, ANGLE))

    # Use Trigger to set speed for better control
    forward_speed = RC.controller.get_trigger(RC.controller.Trigger.RIGHT)
    back_speed = RC.controller.get_trigger(RC.controller.Trigger.LEFT)
    SPEED = (forward_speed - back_speed) if (forward_speed <= 0 or back_speed <= 0) else 0

    RC.drive.set_speed_angle(SPEED, ANGLE)
    print("Speed:",SPEED,"Angle:",ANGLE)

def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # TODO(emi): Print ascii stuff here
    pass

################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    RC.set_start_update(start, update, update_slow)
    RC.go()
