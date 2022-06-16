"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Defines the interface of the Drone module of the racecar_core library.
"""

import abc
import copy
import numpy as np
from nptyping import NDArray


class Drone(abc.ABC):
    """
    Returns the color image captured by the drone's camera.
    """

    # The dimensions of the image in pixels
    _WIDTH: int = 640
    _HEIGHT: int = 480


    def get_drone_image(self) -> NDArray[(480, 640, 3), np.uint8]:
        """
        Returns a deep copy of the current color image captured by the drone camera.

        Returns:
            An array representing the pixels in the image, organized as follows
                0th dimension: pixel rows, indexed from top to bottom.
                1st dimension: pixel columns, indexed from left to right.
                2nd dimension: pixel color channels, in the blue-green-red format.

        Note:
            Each color value ranges from 0 to 255.

            This function returns a deep copy of the captured image. This means that
            we can modify the returned image and it will not change the image returned
            by future calls to get_drone_image().

        Example::

            # Initialize image with a deep copy of the most recent color image captured
            # by the camera
            image = rc.drone.get_drone_image()

            # Store the amount of blue in the pixel on row 3, column 5
            blue = image[3][5][0]
        """
        return copy.deepcopy(self.get_drone_image_no_copy())

    @abc.abstractmethod
    def get_drone_image_async(self) -> NDArray[(480, 640, 3), np.uint8]:
        """
        Returns the current color image without the car in "go" mode.

        Returns:
            An array representing the pixels in the image, organized as follows
                0th dimension: pixel rows, indexed from top to bottom.
                1st dimension: pixel columns, indexed from left to right.
                2nd dimension: pixel color channels, in the blue-green-red format.

        Note:
            Each color value ranges from 0 to 255.

        Warning:
            This function breaks the start-update paradigm and should only be used in
            Jupyter Notebook.

        Example::

            # Initialize image with the most recent color image captured by the camera
            image = rc.drone.get_color_image_async()

            # Store the amount of blue in the pixel on row 3, column 5
            blue = image[3][5][0]
        """
        pass

    @abc.abstractmethod
    def get_drone_height(self) -> float:
        """
        Returns the drone's height.
        """
        pass

    @abc.abstractmethod
    def set_height(self, height: float) -> None:
        """
        Sets the drone's height.

        Args:
            height: The vertical height the drone should fly to.
        """
        pass

    @abc.abstractmethod
    def return_home(self) -> None:
        """
        Lands the drone back on the car
        """
        pass
