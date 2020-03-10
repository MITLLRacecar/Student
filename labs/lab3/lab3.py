"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 3 - Depth Camera
"""

################################################################################
# Imports
################################################################################

import sys

sys.path.insert(0, "../../library")
from racecar_core import *
import racecar_utils as rc_utils

rospy.init_node("racecar")
import cv2 as cv
import numpy as np


################################################################################
# Global variables
################################################################################

rc = Racecar()

# Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels

################################################################################
# Functions
################################################################################


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
        ">> Lab 3 - Depth Camera\n"
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
    depth_image = rc.depth_camera.get_depth_image()
    center = None
    area = 0

    if image is None:
        # If no image is found, center the wheels
        angle = 0
        area = 0 
    
    #TODO: Make the safety feature better
    else:
       if depth_image is not None:
            pix = (rc.depth_camera.get_width()/2, rc.depth_camera.get_height()/2)
            safety_distance = depth_image[pix[1], pix[0]]

            #If the car is within 100 cm of an object have it stop
            if safety_distance > 1000:
               speed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
           
            else:
                speed = 0

    # Calculate angle from left joystick
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

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
    depth_image = rc.depth_camera.get_depth_image()

    if image is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:

        pix = (rc.depth_camera.get_width()/2, rc.depth_camera.get_height()/2)
        sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % ("Depth Topic", pix[0], pix[1], depth_image[pix[1], pix[0]]))
        # Display the image to the screen

        #rc.display.show_image(depth_image)

################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
