#############################
#### Imports
#############################

# General 
import os
import cv2
import numpy as np
from matplotlib import pyplot as plt
from typing import Tuple

# ROS 
try:
    import rospy
    from rospy.numpy_msg import numpy_msg
    from sensor_msgs.msg import LaserScan
    from sensor_msgs.msg import Image
    from ackermann_msgs.msg import AckermannDriveStamped
except:
    print('ROS is not installed')

# iPython Display
import PIL.Image
from io import BytesIO
import IPython.display
import time

#############################
#### Racecar ROS Class
#############################

cap = None
released = True
class Racecar:
    SCAN_TOPIC: str = "/scan"
    IMAGE_TOPIC: str = "/camera"
    DRIVE_TOPIC: str = "/drive"
    
    def __init__(self):
        self.sub_scan = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, callback=self.scan_callback)
        self.sub_image = rospy.Subscriber(self.IMAGE_TOPIC, Image, callback=self.image_callback)
        self.pub_drive = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.last_drive = AckermannDriveStamped()
        self.last_image = None
        self.last_scan = None
    
    def image_callback(self, msg):
        self.last_image = msg.data
        
    def scan_callback(self, msg):
        self.last_scan = msg.ranges
        
    def set_speed(self, speed: float):
        MAX_SPEED = 5
        speed = min(MAX_SPEED, speed)
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        self.last_drive = msg

    def set_angle(self, angle: float):
        MAX_ANGLE = 20
        CONVERSION_FACTOR = 1 / 80
        angle = max(-MAX_ANGLE, min(MAX_ANGLE, angle))
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = angle * CONVERSION_FACTOR
        self.last_drive = msg

    def get_lidar(self):
        return None

    def get_image(self):
        return np.fromstring(self.last_image,dtype=np.uint8).reshape((480,-1,3))[...,::-1]

    def get_acceleration(self) -> Tuple[float, float, float]:
        return (0, 0, 0)

    def get_angular_velocity(self) -> Tuple[float, float, float]:
        return (0, 0, 0)

    def show_image(self, image):
        raise NotImplementedError()

    def show_text(self, text: str, size: float = 12, color: Tuple[float, float, float] = (0, 0, 0)):
        raise NotImplementedError()
    
    def get_mic_amplitude(self, ) -> float:
        return 0

    def set_wav(self, wav):
        raise NotImplementedError()

    def play_wave(self):
        raise NotImplementedError()

    def is_wav_playing(self) -> bool:
        return False

    def get_pin(self, pin: int) -> int:
        return 0

    def set_pin(self, pin: int, value: int):
        raise NotImplementedError()
