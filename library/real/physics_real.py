"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Physics module of the racecar_core library
"""

from physics import Physics

# General
from collections import deque
import numpy as np
from nptyping import NDArray

# ROS
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion


class PhysicsReal(Physics):
    # The ROS topic from which we read imu data
    __ACCEL_TOPIC = "/camera/accel/sample"
    __GYRO_TOPIC = "/camera/gyro/sample"

    # Limit on buffer size to prevent memory overflow
    __BUFFER_CAP = 60

    def __init__(self):
        self.__accel_subscriber = rospy.Subscriber(
            self.__ACCEL_TOPIC, Imu, self.__accel_callback
        )
        self.__gyro_subscriber = rospy.Subscriber(
            self.__GYRO_TOPIC, Imu, self.__gyro_callback
        )

        self.__orientation = np.array([0, 0, 0, 0])
        self.__orientation_buffer = deque()
        self.__acceleration = np.array([0, 0, 0])
        self.__acceleration_buffer = deque()
        self.__angular_velocity = np.array([0, 0, 0])
        self.__angular_velocity_buffer = deque()

    def __accel_callback(self, data):
        new_acceleration = np.array(
            [
                data.linear_acceleration.x,
                data.linear_acceleration.y,
                data.linear_acceleration.z,
            ]
        )

        self.__acceleration_buffer.append(new_acceleration)
        if len(self.__acceleration_buffer) > self.__BUFFER_CAP:
            self.__acceleration_buffer.popleft()

    def __gyro_callback(self, data):
        new_angular_velocity = np.array(
            [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
        )

        self.__angular_velocity_buffer.append(new_angular_velocity)
        if len(self.__angular_velocity_buffer) > self.__BUFFER_CAP:
            self.__angular_velocity_buffer.popleft()

    def __update(self):
        if len(self.__orientation_buffer) > 0:
            self.__orientation = np.mean(self.__orientation_buffer, axis=0)
            self.__orientation_buffer.clear()

        if len(self.__acceleration_buffer) > 0:
            self.__acceleration = np.mean(self.__acceleration_buffer, axis=0)
            self.__acceleration_buffer.clear()

        if len(self.__angular_velocity_buffer) > 0:
            self.__angular_velocity = np.mean(self.__angular_velocity_buffer, axis=0)
            self.__angular_velocity_buffer.clear()

    def get_linear_acceleration(self) -> NDArray[3, np.float32]:
        return np.array(self.__acceleration)

    def get_angular_velocity(self) -> NDArray[3, np.float32]:
        return np.array(self.__angular_velocity)
