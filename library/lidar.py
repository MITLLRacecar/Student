"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Contains the Lidar module of the racecar_core library
"""

import abc
import numpy as np
from nptyping import NDArray


class Lidar(abc.ABC):
    """
    Returns the scan data captured by the Lidar.
    """

    _NUM_SAMPLES: int = 720

    def get_num_samples(self) -> int:
        """
        Returns the length of the ranges array, to check for a valid scan.

        Returns:
            (Int) The number of points collected in each scan.

        Example:
            total_points = rc.lidar.get_length()
        """
        return self._NUM_SAMPLES

    @abc.abstractmethod
    def get_samples(self) -> NDArray[720, np.float32]:
        """
        Returns the array of all the distance value from a single lidar scan.

        Returns:
             (float) The tuple of distance measurements

        Example:
            lidar_ranges = rc.lidar.get_ranges()
        """
        pass
