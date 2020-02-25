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
        self.__publisher = rospy.Publisher(self.__TOPIC, \
            AckermannDriveStamped, queue_size=1)
        self.__message = AckermannDriveStamped()


    def set_speed_angle(self, speed, angle):
        """
        Sets the speed at which the wheels turn and the angle of the front
        wheels

        Inputs:
            speed (float) = the speed from -1.0 to 1.0, with positive for
                forward and negative for reverse
            angle (float) = the angle of the front wheels from -1.0 to 1.0,
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
        MAX_SPEED = 1  # The maximum speed magnitude allowed
        MAX_ANGLE = 1  # The maximum angle magnitude allowed
        SPEED_CONVERSION_FACTOR = 4
        ANGLE_CONVERSION_FACTOR = 1 / 4.0

        speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
        angle = max(-MAX_ANGLE, min(MAX_ANGLE, angle))
        self.__message.drive.speed = speed * SPEED_CONVERSION_FACTOR
        self.__message.drive.steering_angle = angle * \
            ANGLE_CONVERSION_FACTOR

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


    def __update(self):
        """
        Publishes the current drive message
        """
        self.__publisher.publish(self.__message)
