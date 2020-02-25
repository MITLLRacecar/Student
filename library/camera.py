"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Camera module of the racecar_core library
"""

# General
import cv2 as cv
import numpy as np

# ROS
import rospy
from sensor_msgs.msg import Image

class Camera:
    """
    Returns the color images and depth images captured by the camera
    """
    # The ROS topic from which we read camera data
    __TOPIC = "/camera"

    def __init__(self):
        self.__cam = cv.VideoCapture(2)

    def __del__(self):
        self.__cam.release()

    def get_image(self):
        """
        Returns a two dimensional array representing a colored photo

        Output (2D numpy array of triples): A two dimensional array indexed
            from top left to the bottom right representing the pixels in the
            image. Each entry in the array is a triple of the form
            (blue, green, red) representing a single pixel

        Triple format: (blue, green, red)
            blue = the amount of blue at that pixel from 0 (none) to 255 (max)
            green = the amount of green at that pixel from 0 (none) to 255 (max)
            red = the amount of red at that pixel from 0 (none) to 255 (max)
        """
        return self.__cam.read()[1]

    def get_depth_image(self):
        """
        Returns a two dimensional array representing a colored photo with depth
        information

        Output (2D numpy array of quadruples): A two dimensional array indexed
            from top left to the bottom right representing the pixels in the
            image. Each entry in the array is a quadruple of the form
            (blue, green, red, depth) representing a single pixel

        Quadruple format: (blue, green, red, depth)
            blue = the amount of blue at that pixel from 0 (none) to 255 (max)
            green = the amount of green at that pixel from 0 (none) to 255 (max)
            red = the amount of red at that pixel from 0 (none) to 255 (max)
            depth = ???
        """
        #TODO Add depth channel to image gotten from realsense
        return None
