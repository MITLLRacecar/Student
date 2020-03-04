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

sys.path.insert(0, "../../library")
from racecar_core import *

rospy.init_node("racecar")
import cv2 as cv
import numpy as np


################################################################################
# Global variables
################################################################################

rc = Racecar()

# Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_SIZE = 30
# A crop window for the floor directly in front of the car
CROP_BOTTOM = ((400, 0), (rc.camera.get_height(), rc.camera.get_width()))

# Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 50, 50), (110, 255, 255))  # The HSV range for the color blue
# TODO: add HSV ranges for other colors


################################################################################
# Functions
################################################################################


def crop(image, top_left_inclusive, bottom_right_exclusive):
    """
    Crops an image to a rectangle based on the specified pixel points

    Inputs:
        image (2D numpy array of triples): The image to crop
        top_left_inclusive ((int, int)): The (row, column) of the top left pixel
            of the crop rectangle
        bottom_right_exclusive ((int, int)): The (row, column) of the pixel one
            past the bottom right corner of the crop rectangle

    Note: The top_left_inclusive pixel is included in the crop rectangle, but
        the bottom_right_exclusive pixel is not.  This is similar to how the how
        range(1, 4) returns [1, 2, 3] in Python.

    Output (2D numpy array of triples): a cropped version of the image

    Example:
    ```Python
    image = rc.camera.get_image()

    # Crop the image to only keep the top half
    cropped_image = crop(
        image, (0, 0), (rc.camera.get_height() / 2, rc.camera.get_width())
    )
    ```
    """
    # Extract the minimum and maximum pixel rows and columns from the parameters
    r_min, c_min = top_left_inclusive
    r_max, c_max = bottom_right_exclusive

    # Shorten the array to the specified row and column ranges
    return image[r_min:r_max, c_min:c_max]


def find_contours(image, hsv_lower, hsv_upper):
    """
    Finds all contours of the specified color range in the provided image

    Inputs:
        image (2D numpy array of triples): The image in which to find contours,
            with pixels represented in the bgr (blue-green-red) format
        hsv_lower ((int, int, int)): The lower bound for the hue, saturation,
            and value of colors to contour
        hsv_upper ((int, int, int)): The upper bound for the hue, saturation,
            and value of the colors to contour

    Note: Each channel in hsv_lower and hsv_upper ranges from 0 to 255

    Output ([contours]): a list of contours around the specified color ranges
        found in the provided image

    Example:
    ```Python
    # Define the lower and upper hsv ranges for the color blue
    BLUE_HSV_MIN = (90, 50, 50)
    BLUE_HSV_MAX = (110, 255, 255)

    # Extract contours around all blue portions of the current image
    contours = find_contours(rc.camera.get_image(), BLUE_HSV_MIN, BLUE_HSV_MAX)
    ```
    """
    # Convert the image from a blue-green-red pixel representation to a
    # hue-saturation-value representation
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # Create a mask based on the pixels in the image with hsv values that
    # fall between HSV_lower and HSV_upper
    mask = cv.inRange(hsv_image, hsv_lower, hsv_upper)

    # Find and return a list of all contours of this mask
    return cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[1]


def get_largest_contour(contours):
    """
    Extracts the largest contour from a list of contours with an area
    greater than MIN_CONTOUR_SIZE

    Inputs:
        contours ([contour]): A list of contours found in an image

    Outputs (contour or None): The largest contour from the list, or None if no
        contour was larger than MIN_CONTOUR_SIZE

    Example:
    ```Python
    # Extract the blue contours
    BLUE_HSV_MIN = (90, 50, 50)
    BLUE_HSV_MAX = (110, 255, 255)
    contours = find_contours(rc.camera.get_image(), BLUE_HSV_MIN, BLUE_HSV_MAX)

    # Find the largest contour
    largest_contour = get_largest_contour(contours)
    ```
    """
    # Check that the list contains at least one contour
    if len(contours) == 0:
        return None

    # Find and return the largest contour if it is larger than MIN_CONTOUR_SIZE
    greatest_contour = max(contours, key=cv.contourArea)
    if cv.contourArea(greatest_contour) < MIN_CONTOUR_SIZE:
        return None

    return greatest_contour


def draw_contour(image, contour, color=(0, 255, 0)):
    """
    Draws a contour on a copy of the provided image

    Inputs:
        image (2D numpy array of triples): The image on which to draw the
            contour
        contour (contour): A contour to draw on the image
        color ((int, int, int)): The color to draw the contour, specified as
            blue-green-red channels each ranging from 0 to 255

    Output (2D numpy array ): a copy of image with the contour drawn on it
    """
    return cv.drawContours(np.copy(image), [contour], 0, color, 3)


def get_center(contour):
    """
    Finds the center of a contour from an image

    Inputs:
        contour (contour): The contour of which to find the center

    Output ((int, int)): The (row, column) of the pixel at the center of the 
        contour

    Note: Returns a None if the contour parameter is None or contains no pixels

    Example:
    ```Python
    # Extract the largest blue contour
    BLUE_HSV_MIN = (90, 50, 50)
    BLUE_HSV_MAX = (110, 255, 255)
    contours = find_contours(rc.camera.get_image(), BLUE_HSV_MIN, BLUE_HSV_MAX)
    largest_contour = get_largest_contour(contours)

    # Find the center of this contour if it exists
    if (largest_contour is not None):
        center = get_center(largest_contour)
    ```
    """
    # Verify that we were passed a valid contour
    if contour is None:
        return None

    M = cv.moments(contour)

    # Check that the contour is not empty
    # (M["m00"] is the number of pixels in the contour)
    if M["m00"] <= 0:
        return None

    # Compute and return the center of mass of the contour
    return (M["m01"] / M["m00"], M["m10"] / M["m00"])


def get_area(contour):
    """
    Finds the area of a contour from an image

    Inputs:
        contour (contour): The contour of which to measure the area

    Output (float): The number of pixels contained within the contour, or 0 if
        an invalid contour is provided

    Example:
    ```Python
    # Extract the largest blue contour
    BLUE_HSV_MIN = (90, 50, 50)
    BLUE_HSV_MAX = (110, 255, 255)
    contours = find_contours(rc.camera.get_image(), BLUE_HSV_MIN, BLUE_HSV_MAX)
    largest_contour = get_largest_contour(contours)

    # Find the area of this contour (will evaluate to 0 if no contour was found)
    area = get_contour_area(contour)
    ```
    """
    # Verify that we were passed a valid contour
    if contour is None:
        return 0

    return cv.contourArea(contour)


def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab 2 - Image processing\n"
        "\n"
        "Controlls:\n"
        "   Right trigger = control forward speed\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle

    image = rc.camera.get_image()
    center = None
    area = 0

    # TODO: Implement a way to cycle through following multiple colors of tape

    if image is None:
        # If no image is found, center the wheels
        angle = 0
        center = None
        area = 0
    else:
        # Crop the image to only show the floor in front of the car
        cropped_image = crop(image, CROP_BOTTOM[0], CROP_BOTTOM[1])

        # Find all of the blue contours
        blueContours = find_contours(cropped_image, BLUE[0], BLUE[1])

        # Find the center and area of the largest blue contour
        largestBlueContour = get_largest_contour(blueContours)
        center = get_center(largestBlueContour)
        area = get_area(largestBlueContour)

    # Choose an angle based on center
    # If we could not find a contour center, keep the previous angle
    if center is not None:
        # TODO: Implement a smoother way for the car to follow the line
        if center[1] < rc.camera.get_width() / 2:
            angle = 1
        else:
            angle = -1

    # Use the right trigger to control the car's forward speed
    speed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if center is None:
            print("No contour found")
        else:
            print("Center:", center, "Area:", area)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # To help debug, update_slow does the following:
    # 1. Prints a line of ascii text to the console denoting the area of the
    #    contour and where the car sees the line
    # 2. Shows the current image to the screen with the largest contour drawn
    #    on top in bright green

    image = rc.camera.get_image()
    if image is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # Crop the image to only show the floor in front of the car
        cropped_image = crop(image, CROP_BOTTOM[0], CROP_BOTTOM[1])

        # Find all of the blue contours
        blueContours = find_contours(cropped_image, BLUE[0], BLUE[1])

        # Find the center and area of the largest blue contour
        largestBlueContour = get_largest_contour(blueContours)
        center = get_center(largestBlueContour)
        area = get_area(largestBlueContour)

        # If an image is found but no contour is found, print all dashes
        if center is None:
            print("-" * 32 + " : area = " + str(area))

        # Otherwise, print a line of dashes with a | where the line is seen
        else:
            s = ["-"] * 32
            s[int(center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(area))

            # Draw the contour onto the image
            cropped_image = draw_contour(cropped_image, largestBlueContour)

        # Display the image to the screen
        rc.display.show_image(cropped_image)


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
