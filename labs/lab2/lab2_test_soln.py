"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 2 - Line/Cone following
"""

################################################################################
# Imports
################################################################################

import cv2 as cv

################################################################################
# Global variables
################################################################################

CAMERA = cv.VideoCapture(0)
MIN_CONTOUR_SIZE = 30
LINE_COLOR_PRIORITY = list()
SPEED = 5
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
    img = crop(img, (400, 0), (480, 640))
    cv.imwrite('test_out_cropped.png', img)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, HSV_lower, HSV_upper)
    cv.imwrite('test_out_mask.png', mask)
    contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
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
    turn_factor = 30

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
    max_angle = -turn_factor
    if speed < 0:
        max_angle = turn_factor
    return ratio*max_angle

def start():
    """
    This function is run once every time the start button is pressed
    """
    global SPEED
    global ANGLE
    # Here we write a mask for each color of tape we care about, and
    # put in a bunch of line colors with different priorities
    orange = ((0,0,0), (30, 255, 255))
    green = ((0,0,0), (255, 255, 255))
    blue = ((0,0,0), (255, 255, 255))
    LINE_COLOR_PRIORITY.append(green)
    LINE_COLOR_PRIORITY.append(blue)
    LINE_COLOR_PRIORITY.append(orange)

    # We also start the car driving
    print(f"Speed: {SPEED}, Angle: {ANGLE}")

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global SPEED
    global ANGLE
    for color_bound in LINE_COLOR_PRIORITY:
        # We get the upper and lower boudns for the color
        # that we care about
        hsv_lower, hsv_upper = color_bound
        _,image = CAMERA.read()
        cv.imwrite('test_out_original.png', image)
        exists, contour = contours_exist(find_contours(image, hsv_lower, hsv_upper))
        if not exists:
            break
        ANGLE = get_angle(contour, (SPEED, ANGLE))
        print(f"Speed: {SPEED}, Angle: {ANGLE}")
    

################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    start()
    update()
