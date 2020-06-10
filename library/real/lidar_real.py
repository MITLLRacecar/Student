"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Lidar module of the racecar_core library
"""

from lidar import Lidar

# General
import numpy as np
from collections import deque

# ROS
import rospy
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import threading


class LidarReal(Lidar):
    # The ROS topic from which we get Lidar data
    __SCAN_TOPIC = "/scan"

    def __init__(self):
        self.__scan_sub = rospy.Subscriber(
            self.__SCAN_TOPIC, LaserScan, self.__scan_callback
        )
        self._event = threading.Event()
        self.__samples = None

    def __scan_callback(self, data):
        self.__samples = list(data.ranges)
        self._event.set()

    def get_samples(self):
        # TODO: Handle threading
        return self.__samples
