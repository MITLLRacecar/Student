#############################
#### Imports
#############################

# General 
import os
# from typing import Tuple

# ROS 
import rospy
# from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped

#############################
#### Racecar ROS Class
#############################

class Racecar:
    def __init__(self):
        self.drive = self.Drive()

    def run(self, start, update):
        FRAMES_PER_SECOND = 60
        start()
        timer = rospy.Rate(FRAMES_PER_SECOND)
        for i in range(300):
            update()
            self.__publish()
            timer.sleep()

    def __publish(self):
        self.drive.publish()

    class Drive:
        __TOPIC = "/drive"

        def __init__(self):
            self.__publisher = rospy.Publisher(self.__TOPIC, AckermannDriveStamped, queue_size=1)
            self.__message = AckermannDriveStamped()

        def set_speed(self, speed):
            MAX_SPEED = 5
            speed = min(MAX_SPEED, speed)
            self.__message.drive.speed = speed

        def set_angle(self, angle):
            MAX_ANGLE = 20
            CONVERSION_FACTOR = 1 / 80
            angle = max(-MAX_ANGLE, min(MAX_ANGLE, angle))
            self.__message.drive.steering_angle = angle * CONVERSION_FACTOR

        def publish(self):
            self.__publisher.publish(self.__message)

    
    # class Physics:
    #     def get_acceleration():

    # class Physics:
    #     def get_acceleration(self) -> Tuple[float, float, float]:
    #         return (0, 0, 0)

    #     def get_angular_velocity(self) -> Tuple[float, float, float]:
    #         return (0, 0, 0)

    #     def get_speed(self) -> float:
    #         return 0

    # class Camera:
    #     def get_image(self):
    #         return np.fromstring(self.last_image,dtype=np.uint8).reshape((480,-1,3))[...,::-1]

    #     def get_depth_image(self):
    #         return None

    # class Lidar:
    #     def get_raw(self):
    #         return None
        
    #     def get_map(self):
    #         return None    

    # class GPIO:
    #     def get_pin(self, pin: int) -> int:
    #         return 0

    #     def set_pin(self, pin: int, value: int):
    #         raise NotImplementedError()


    # class Controller:
    #     def is_pressed(self, button) -> bool:
    #         return False

    #     def get_trigger(self, trigger) -> float:
    #         return 0

    #     def get_joystick(self, joystick) -> Tuple[float, float]:
    #         return (0, 0)

    # class Display:

    # class Sound: 


        

    # def image_callback(self, msg):
    #     self.last_image = msg.data
        
    # def scan_callback(self, msg):
    #     self.last_scan = msg.ranges
        
    # def get_lidar(self):
    #     return None



    # def show_image(self, image):
    #     raise NotImplementedError()

    # def show_text(self, text: str, size: float = 12, color: Tuple[float, float, float] = (0, 0, 0)):
    #     raise NotImplementedError()
    
    # def get_mic_amplitude(self, ) -> float:
    #     return 0

    # def set_wav(self, wav):
    #     raise NotImplementedError()

    # def play_wave(self):
    #     raise NotImplementedError()

    # def is_wav_playing(self) -> bool:
    #     return False