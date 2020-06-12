"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Defines the interface for the Lidar module of the racecar_core library.
"""

import abc
import numpy as np
from nptyping import NDArray


class Lidar(abc.ABC):
    """
    Returns the scan data captured by the Lidar.
    """

    # The number of samples in a full Lidar scan.
    _NUM_SAMPLES: int = 720

    def get_num_samples(self) -> int:
        """
        Returns the number of samples in a full LIDAR scan.

        Returns:
            The number of points collected in a complete scan.

        Example:
            total_points = rc.lidar.get_num_samples()
        """
        return self._NUM_SAMPLES

    @abc.abstractmethod
    def get_samples(self) -> NDArray[720, np.float32]:
        """
        Returns an array containing the distance values of each sample in a full scan.

        Returns:
            An array of distance measurements in cm.

        Note:
            Samples are in clockwise order, with the 0th sample directly in front of the
            car.  Each sample is an equal angle appart.


        Example:
            # Access the most recent lidar scan.
            lidar_ranges = rc.lidar.get_ranges()

            # Get the distance of the measurement directly in front of the car
            forward_distance = lidar_ranges[0]
        """
        pass
