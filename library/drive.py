"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Drive module of the racecar_core library
"""

import rospy
import numbers
from ackermann_msgs.msg import AckermannDriveStamped

import sys

sys.path.insert(0, "../../library")
import racecar_utils as rc_utils


class Drive:
    """
    Controls the car's movement by allowing the user to set the state
    associated with speed and turning
    """

    # The ROS topic to which we publish drive messages
    __TOPIC = "/drive"

    # PWM constants
    __PWM_TURN_RIGHT = 3000
    __PWM_TURN_LEFT = 9000
    __PWM_SPEED_MIN = 3000
    __PWM_SPEED_MAX = 9000

    def __init__(self):
        self.__publisher = rospy.Publisher(
            self.__TOPIC, AckermannDriveStamped, queue_size=1
        )
        self.__message = AckermannDriveStamped()
        self.__max_speed_scale_factor = (0.25, 0.33)

    def set_speed_angle(self, speed, angle):
        """
        Sets the speed at which the wheels turn and the angle of the front wheels.

        Example:
            if counter < 1:
                # Drive forward at full speed
                rc.drive.set_speed_angle(1, 0)
            elif counter < 2:
                # Drive fully to the left at full speed
                rc.drive.set_speed_angle(1, -1)
            else:
                # Drive 70% to the right at half speed
                rc.drive.set_speed_angle(0.5, 0.7)

        Args:
            speed: (float) The speed from -1.0 to 1.0, with positive for
                forward and negative for reverse.
            angle: (float) The angle of the front wheels from -1.0 to 1.0,
                    with positive for right negative for left.
        """
        assert (
            isinstance(speed, numbers.Number) and -1.0 <= speed <= 1.0
        ), "speed must be a number between -1.0 and 1.0 inclusive"
        assert (
            isinstance(speed, numbers.Number) and -1.0 <= angle <= 1.0
        ), "angle must be a number between -1.0 and 1.0 inclusive"

        # Scale speed by __max_speed_scale_factor
        speedScaled = (
            speed * self.__max_speed_scale_factor[0]
            if speed > 0
            else self.__max_speed_scale_factor[1]
        )

        self.__message.drive.speed = rc_utils.remap_range(
            speedScaled, -1.0, 1.0, self.__PWM_SPEED_MIN, self.__PWM_SPEED_MAX,
        )

        self.__message.drive.steering_angle = rc_utils.remap_range(
            angle, -1.0, 1.0, self.__PWM_TURN_LEFT, self.__PWM_TURN_RIGHT,
        )

    def stop(self):
        """
        Brings the car to a stop and points the front wheels forward.

        Example:
            # Stops the car if the counter is greater than 5
            if counter > 5:
                rc.drive.stop()
        """
        self.set_speed_angle(0, 0)

    def set_max_speed_scale_factor(self, scale_factor):
        """
        Sets the maximum speed in the forward and backward direction.

        Args:
            scale_factor: (float, float) The maximum speed scale factor for
                forward and backward, each ranging from 0.0 to 1.0.

        Note:
            The RACECAR motor is naturally faster forward than backward, so we
            recommended using a larger scale factor in the backward direction
            to compensate.

        Example:
            # Update the max speed scale factor to 0.5 forward, 0.7 backward
            rc.set_max_speed_scale_factor((0.5, 0.7))
        """
        assert (
            len(scale_factor) == 2
        ), "scale_factor must be a two element tuple of floats"
        assert (
            len(
                filter(
                    scale_factor,
                    lambda x: isinstance(x, numbers.Number) and 0.0 <= x <= 1.0,
                )
            )
            == 2
        ), "both entries of scale_factor must be numbers between 0.0 and 1.0 inclusive"

        self.__max_speed_scale_factor = scale_factor

    def __update(self):
        """
        Publishes the current drive message.
        """
        self.__publisher.publish(self.__message)
