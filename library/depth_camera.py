"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the depth camera module of the racecar_core library
"""

# General
import cv2 as cv
import numpy as np

# ROS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class DepthCamera:
    """
    Returns the color images and depth images captured by the camera
    """

    # The ROS topic from which we read camera data
    __TOPIC = "/camera/depth/image_rect_raw"

    # The dimensions of the image in pixels, as (rows, columns)
    __DIMENSIONS = (480, 640)

    def __init__(self):
        #self.__cam = cv.VideoCapture(2)
        self.depth_pub = rospy.Publisher("/camera/depth/image_rect_raw", Image, queue_size=10)
        self.depth_bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.depth_image = None

    def depth_callback(self, data):
        try:
           cv_depth_image = self.depth_bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
           print(e)

        #(rows,cols,channels) = cv_depth_image.shape

        try:
           self.depth_pub.publish(self.depth_bridge.cv2_to_imgmsg(cv_depth_image, "16UC1"))
        except CvBridgeError as e:
           print(e)

        self.depth_image = cv_depth_image

    def __del__(self):
        self.__cam.release()

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

        Example:
        ```Python
        # Initialize depth_image with the most recent depth image captured
        # by the camera
        depth_image = rc.camera.get_depth_image()
        ```
        """
        return self.depth_image

    def get_width(self):
        """
        Returns the width of the captured images

        Output (int): The width (number of pixel columns) of the captured images

        Example:
        ```Python
        image = rc.camera.get_image()

        # Access the top right pixel of the image
        top_right_pixel = image[0, rc.camera.get_width() - 1]
        ```
        """
        return self.__DIMENSIONS[1]

    def get_height(self):
        """
        Returns the height of the captured images

        Output (int): The height (number of pixel rows) of the captured images

        Example:
        ```Python
        image = rc.camera.get_image()

        # Access the top bottom left pixel of the image
        bottom_left_pixel = image[rc.camera.get_height() - 1, 0]
        ```
        """
        return self.__DIMENSIONS[0]
