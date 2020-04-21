"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Drive module of the racecar_core library
"""

# ROS
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

# Constants, do not modify these
PWM_TURN_RIGHT = 3000
PWM_TURN_LEFT = 9000
PWM_SPEED_MIN = 3000
PWM_SPEED_MAX = 9000

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
        
        # Config
        self.__max_speed = 1.0
        self.__max_turn = 1.0
        self.__max_speed_scale_factor = (0.25, 0.33)

    @staticmethod
    def __remap_to_range(val, old_min, old_max, new_min, new_max):
        """
        Helper function here being used to remap a value from a given range to a new range.

        Inputs:
            val (number): number in old range to be rescaled
            old_min (number): 'lower' bound of old range
            old_max (number): 'upper' bound of old range
            new_min (number): 'lower' bound of new range
            new_max (number): 'upper' bound of new range

        Note: min need not be less than max in general.
        Flipping the direction will cause the sign of the mapping to flip (examples below).

        Example:

            >>> self.__remap_to_range(5,0,10,0,50)
            25

            >>> self.__remap_to_range(5,0,20,1000,900)
            975

        """
        old_span = old_max-old_min
        new_span = new_max-new_min
        return new_min + new_span*(float(val-old_min)/float(old_span))


    def set_speed_angle(self, speed, angle):
        """
        Sets the speed at which the wheels turn and the angle of the front wheels.

        Args:
            speed: (float) The speed from -1.0 to 1.0, with positive for
                forward and negative for reverse.
            angle: (float) The angle of the front wheels from -1.0 to 1.0,
                    with positive for right negative for left.

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
        """
        assert (
            -1.0 <= speed <= 1.0
        ), "speed must be a float between -1.0 and 1.0 inclusive"
        assert (
            -1.0 <= angle <= 1.0
        ), "angle must be a float between -1.0 and 1.0 inclusive"
        self.__message.drive.steering_angle = self.__remap_to_range(angle, -self.__max_turn, self.__max_turn, PWM_TURN_LEFT, PWM_TURN_RIGHT)
        self.__message.drive.speed = speed * (
            self.__max_speed_scale_factor[0]
            if speed > 0
            else self.__max_speed_scale_factor[1]
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
            0.0 <= scale_factor[0] <= 1.0 and 0.0 <= scale_factor[1] <= 1.0
        ), "both entries of scale_factor must be a float between 0.0 and 1.0 inclusive"

        self.__max_speed_scale_factor = scale_factor

    def __update(self):
        """
        Publishes the current drive message.
        """
        self.__publisher.publish(self.__message)
