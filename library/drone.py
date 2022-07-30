"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2022

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


    def get_image(self) -> NDArray[(480, 640, 3), np.uint8]:
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
            by future calls to get_image().

        Example::

            # Initialize image with a deep copy of the most recent color image captured
            # by the camera
            image = rc.drone.get_image()

            # Store the amount of blue in the pixel on row 3, column 5
            blue = image[3][5][0]
        """
        return copy.deepcopy(self.get_image_no_copy())

    @abc.abstractmethod
    def get_image_no_copy(self) -> NDArray[(480, 640, 3), np.uint8]:
        """
        Returns a direct reference to the color image captured by the drone's camera.

        Returns:
            An array representing the pixels in the image, organized as follows
                0th dimension: pixel rows, indexed from top to bottom.
                1st dimension: pixel columns, indexed from left to right.
                2nd dimension: pixel color channels, in the blue-green-red format.

        Warning:
            Do not modify the returned image. The returned image is a reference to the
            captured image, so any changes will also affect the images returned by any
            future calls to get_image() or get_image_no_copy().

        Note:
            Each color value ranges from 0 to 255.

            Unlike get_image(), this function does not create a deep copy of the
            captured image, and is therefore more efficient.

        Example::

            # Initialize image with a direct reference to the recent color image
            # captured by the drone's camera
            image = rc.drone.get_image()

            # Store the amount of blue in the pixel on row 3, column 5
            blue = image[3][5][0]

            # We can safely call crop because it does not modify the source image
            cropped_image = rc_utils.crop(image, (0, 0), (10, 10))

            # However, if we wish to draw on the image, we must first create a manual
            # copy with copy.deepcopy()
            image_copy = copy.deepcopy(image)
            modified_image = rc_utils.draw_circle(image_copy, (50, 50))
        """
        pass

    @abc.abstractmethod
    def get_image_async(self) -> NDArray[(480, 640, 3), np.uint8]:
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
            image = rc.drone.get_image_async()

            # Store the amount of blue in the pixel on row 3, column 5
            blue = image[3][5][0]
        """
        pass

    @abc.abstractmethod
    def get_height(self) -> float:
        """
        Returns the height of the drone.

        Returns:
            A float that represents the height of the drone.

        Example::

            # Store the drone's height in the variable drone_height
            drone_height = rc.drone.get_height()
        """
        pass

    @abc.abstractmethod
    def set_height(self, height: float) -> None:
        """
        Sets the drone's height.

        Args:
            height: The vertical height the drone should fly to.

        Example::

            # Have the drone fly to an altitude of 100 meters.
            rc.drone.set_height(100)
        """
        pass

    @abc.abstractmethod
    def return_to_car(self) -> None:
        """
        Lands the drone back on the car

        Example::

            # Have the drone land back on the car.
            rc.drone.return_to_car()
        """
        pass
