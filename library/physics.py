"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Defines the interface of the Physics module of the racecar_core library
"""

import abc
import numpy as np
from nptyping import NDArray


class Physics(abc.ABC):
    """
    Returns the linear acceleration and angular velocity measured by the IMU and the position measured by the GPS.
    """

    @abc.abstractmethod
    def get_linear_acceleration(self) -> NDArray[3, np.float32]:
        """
        Returns a 3D vector containing the car's linear acceleration.

        Returns:
            The average linear acceleration of the car along the (x, y, z) axes during
            the last frame in m/s^2.

        Note:
            The x axis points out of the right of the car.
            The y axis points directly up (perpendicular to the ground).
            The z axis points out of the front of the car.

        Example::

            # accel stores the average acceleration over the previous frame
            accel = rc.physics.get_linear_acceleration()

            # forward accel stores acceleration in the forward direction.  This will be
            # positive when the car accelerates, and negative when it decelerates.
            forward_accel = accel[2]
        """
        pass

    @abc.abstractmethod
    def get_angular_velocity(self) -> NDArray[3, np.float32]:
        """
        Returns a 3D vector containing the car's angular velocity.

        Returns:
            The average angular velocity of the car along the (x, y, z) axes during the
            last frame in rad/s.

        Note:
            The x axis (pitch) points out of the right of the car.
            The y axis (yaw) points directly up (perpendicular to the ground).
            The z axis (roll) points out of the front of the car.
            Rotation sign uses the right hand rule. For example, when the car turns to
            the left, it has a positive angular velocity along the y axis.


        Example::

            # ang_vel stores the average angular velocity over the previous frame
            ang_vel = rc.physics.get_angular_velocity()

            # yaw stores the yaw of the car, which is positive when it turns to the left
            # and negative when it turns to the right.
            yaw = ang_vel[1]
        """
        pass

    @abc.abstractmethod
    def get_position(self) -> NDArray[3, np.float32]:
        """
        Returns a 3D vector containing the car's position as x, y, and z coordinates.

        Returns:
            The position of the car along the (x, y, z) axes during
            the last frame in m.

        Note:
            The x axis points out of the right of the car.
            The y axis points out of the top of the car.
            The z axis points out of the front of the car.

        Example::

            # position_vector stores the position over the previous frame
            position_vector = rc.physics.get_position()

            x_position = position_vector[0]
            y_position = position_vector[1]
            z_position = position_vector[2]
        """
        pass

