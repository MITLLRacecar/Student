"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Drive module of the racecar_core library
"""

# ROS
import rospy
from ackermann_msgs.msg import AckermannDriveStamped


class Drive:
    """
    Controls the car's movement by allowing the user to set the state
    associated with speed and turning
    """

    # The ROS topic to which we publish drive messages
    __TOPIC = "/drive"

    def __init__(self):
        self.__publisher = rospy.Publisher(
            self.__TOPIC, AckermannDriveStamped, queue_size=1
        )
        self.__message = AckermannDriveStamped()
        self.__max_speed_scale_factor = 0.25

    def set_speed_angle(self, speed, angle):
        """
        Sets the speed at which the wheels turn and the angle of the front
        wheels

        Inputs:
            speed (float): The speed from -1.0 to 1.0, with positive for
                forward and negative for reverse
            angle (float): The angle of the front wheels from -1.0 to 1.0,
                    with positive for right negative for left

        Example:
        ```Python
        if counter < 1:
            # Drive forward at full speed
            rc.drive.set_speed_angle(1, 0)
        elif counter < 2:
            # Drive fully to the left at full speed
            rc.drive.set_speed_angle(1, -1)
        else:
            # Drive 70% to the right at half speed
            rc.drive.set_speed_angle(0.5, 0.7)
        ```
        """
        assert (
            -1.0 <= speed <= 1.0
        ), "speed must be a float between -1.0 and 1.0 inclusive"
        assert (
            -1.0 <= angle <= 1.0
        ), "angle must be a float between -1.0 and 1.0 inclusive"

        self.__message.drive.speed = speed * self.__max_speed_scale_factor
        self.__message.drive.steering_angle = angle

    def stop(self):
        """
        Brings the car to a stop and points the front wheels forward

        Example:
        ```Python
        # Stops the car if the counter is greater than 5
        if counter > 5:
            rc.drive.stop()
        ```
        """
        self.set_speed_angle(0, 0)

    def set_max_speed_scale_factor(self, scale_factor):
        assert (
            0.0 <= scale_factor <= 1.0
        ), "max_speed_scale_factor must be a float between 0.0 and 1.0 inclusive"
        self.__max_speed_scale_factor = scale_factor

    def __update(self):
        """
        Publishes the current drive message
        """
        self.__publisher.publish(self.__message)
