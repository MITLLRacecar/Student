"""
Copyright Harvey Mudd College
MIT License
Spring 2020
Contains the Lidar module of the racecar_core library
"""

# General
import cv2 as cv
import numpy as np

# ROS
import rospy
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError

class Lidar:
    """
    Returns robot position and scan data
    """

    # The ROS topic from which we get lidar data
    __SCAN_TOPIC = "/scan"
    
    def __init__(self):
        #rospy.init_node('scan_values')
        self.__scan_sub = rospy.Subscriber(self.__SCAN_TOPIC, LaserScan, self.__scan_callback)
        #rospy.spin()
        self.__length = None
    
    def __scan_callback(self, data):
        self.__length = len(data.ranges)
        print("ds")
        return(msg.ranges)
        print(msg.ranges[0])
        print(msg.ranges[360])
        print(msg.ranges[720])

    def get_length(self):
        return self.__length


    #length = self.get_length()
    #print(length)
