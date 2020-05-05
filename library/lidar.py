"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Lidar module of the racecar_core library
"""

# General
import numpy as np
from collections import deque

# ROS
import rospy
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import threading


class Lidar:
    """
    Returns the scan data captured by the Lidar.
    """

    # The ROS topic from which we get Lidar data
    __SCAN_TOPIC = "/scan"

    def __init__(self):
        self.__scan_sub = rospy.Subscriber(
            self.__SCAN_TOPIC, LaserScan, self.__scan_callback
        )
        self._event = threading.Event()
        self.__length = 0
        self.__ranges = ()

    def __scan_callback(self, data):

        self.__length = len(data.ranges)
        self.__ranges = list(data.ranges)
        self._event.set()

    def get_length(self, timeout=None):
        """
        Returns the length of the ranges array, to check for a valid scan.

        Returns:
            (Int) The number of points collected in each scan.

        Example:
            total_points = rc.lidar.get_length()
        """
        self._event.wait(timeout)
        return self.__length

    def get_ranges(self, timeout=None):
        """
        Returns the array of all the distance value from a single lidar scan.

        Returns:
             (float) The tuple of distance measurements

        Example:
            lidar_ranges = rc.lidar.get_ranges()
        """
        self._event.wait(timeout)
        return self.__ranges
