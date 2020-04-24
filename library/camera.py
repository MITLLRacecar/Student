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
from cv_bridge import CvBridge, CvBridgeError


class Camera:
    """
    Returns the color images and depth images captured by the camera.
    """

    # The ROS topic from which we read camera data
    __COLOR_TOPIC = "/camera/color/image_raw"
    __DEPTH_TOPIC = "/camera/depth/image_rect_raw"

    # The dimensions of the image in pixels, as (rows, columns)
    __DIMENSIONS = (480, 640)

    def __init__(self):
        self.__bridge = CvBridge()

        self.__color_image_sub = rospy.Subscriber(
            self.__COLOR_TOPIC, Image, self.__color_callback
        )
        self.__color_image = None
        self.__color_image_new = None

        self.__depth_image_sub = rospy.Subscriber(
            self.__DEPTH_TOPIC, Image, self.__depth_callback
        )
        self.__depth_image = None
        self.__depth_image_new = None

    def __color_callback(self, data):
        try:
            cv_color_image = self.__bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.__color_image_new = cv_color_image

    def __depth_callback(self, data):
        try:
            cv_depth_image = self.__bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

        self.__depth_image_new = cv_depth_image

    def __update(self):
        self.__depth_image = self.__depth_image_new
        self.__color_image = self.__color_image_new

    def get_image(self):
        """
        Returns the previous color image captured by the camera.

        Returns:
            (2D numpy array of triples) A two dimensional array indexed
            from top left to the bottom right representing the pixels in the
            image. Each entry in the array is a triple of the form
            (blue, green, red) representing a single pixel.

        Note:
            Triple format = (blue, green, red), with
                blue = the amount of blue at that pixel from 0 (none) to 255 (max)
                green = the amount of green at that pixel from 0 (none) to 255 (max)
                red = the amount of red at that pixel from 0 (none) to 255 (max)

        Example:
            # Initialize image with the most recent image captured by the camera
            image = rc.camera.get_image()
        """
        return self.__color_image

    def get_depth_image(self):
        """
        Returns the previous depth image captured by the camera.

        Returns:
            (2D numpy array of floats) A two dimensional array indexed
            from top left to the bottom right representing the pixels in the
            image. The value of each pixel is the distance detected at that point
            in millimeters.

        Example:
            # Initialize depth_image with the most recent depth image captured
            # by the camera
            depth_image = rc.camera.get_depth_image()
        """
        return self.__depth_image

    def get_width(self):
        """
        Returns the width of the captured images.

        Returns:
            (int) The width (number of pixel columns) of the captured images.

        Example:
            image = rc.camera.get_image()

            # Access the top right pixel of the image
            top_right_pixel = image[0, rc.camera.get_width() - 1]
        """
        return self.__DIMENSIONS[1]

    def get_height(self):
        """
        Returns the height of the captured images.

        Returns:
            (int) The height (number of pixel rows) of the captured images.

        Example:
            image = rc.camera.get_image()

            # Access the top bottom left pixel of the image
            bottom_left_pixel = image[rc.camera.get_height() - 1, 0]
        """
        return self.__DIMENSIONS[0]

    def _get_image_async(self):
        """
        Jupyter only - returns the current color image captured by the camera.
F
        Returns:
            (2D numpy array of triples) A two dimensional array indexed
            from top left to the bottom right representing the pixels in the
            image. Each entry in the array is a triple of the form
            (blue, green, red) representing a single pixel.

        Note:
            Triple format = (blue, green, red), with
                blue = the amount of blue at that pixel from 0 (none) to 255 (max)
                green = the amount of green at that pixel from 0 (none) to 255 (max)
                red = the amount of red at that pixel from 0 (none) to 255 (max)

        Warning:
            This function violates the start update paradigm and should only be used
            in Jupyter Notebooks.

        Example:
            # Initialize image with the most recent image captured by the camera
            image = rc.camera.get_image()
        """
        return self.__color_image_new

    def _get_depth_image_async(self):
        """
        Jupyter only - returns the previous depth image captured by the camera.

        Returns:
            (2D numpy array of floats) A two dimensional array indexed
            from top left to the bottom right representing the pixels in the
            image. The value of each pixel is the distance detected at that point
            in millimeters.

        Warning:
            This function violates the start update paradigm and should only be used
            in Jupyter Notebooks.

        Example:
            # Initialize depth_image with the most recent depth image captured
            # by the camera
            depth_image = rc.camera.get_depth_image()
        ```
        """
        return self.__depth_image_new
