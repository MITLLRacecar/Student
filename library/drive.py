"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Contains the Drive module of the racecar_core library
"""

import abc


class DriveBase(abc.ABC):
    @classmethod
    @abc.abstractmethod
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
        pass

    @classmethod
    @abc.abstractmethod
    def stop(self):
        """
        Brings the car to a stop and points the front wheels forward.

        Example:
            # Stops the car if the counter is greater than 5
            if counter > 5:
                rc.drive.stop()
        """
        pass

    @classmethod
    @abc.abstractmethod
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
        pass
