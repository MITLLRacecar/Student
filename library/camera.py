"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Defines the interface of the Camera module of the racecar_core library.
"""

import abc
import numpy as np
from nptyping import NDArray


class Camera(abc.ABC):
    """
    Returns the color images and depth images captured by the camera.
    """

    # The dimensions of the image in pixels
    _WIDTH: int = 640
    _HEIGHT: int = 480

    # Maximum range of the depth camera (in cm)
    _MAX_RANGE = 1200

    @abc.abstractmethod
    def get_color_image(self) -> NDArray[(480, 640, 3), np.uint8]:
        """
        Returns the current color image captured by the camera.

        Returns:
            An array representing the pixels in the image, organized as follows
                0th dimension: pixel rows, indexed from top to bottom.
                1st dimension: pixel columns, indexed from left to right.
                2nd dimension: pixel color channels, in the blue-green-red format.

        Note:
            Each color value ranges from 0 to 255.

        Example::

            # Initialize image with the most recent color image captured by the camera
            image = rc.camera.get_color_image()

            # Store the amount of blue in the pixel on row 3, column 5
            blue = image[3][5][0]
        """
        pass

    @abc.abstractmethod
    def get_color_image_async(self) -> NDArray[(480, 640, 3), np.uint8]:
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
            image = rc.camera.get_color_image_async()

            # Store the amount of blue in the pixel on row 3, column 5
            blue = image[3][5][0]
        """
        pass

    @abc.abstractmethod
    def get_depth_image(self) -> NDArray[(480, 640), np.float32]:
        """
        Returns the current depth image captured by the camera.

        Returns:
            A two dimensional array indexed from top left to the bottom right storing
            the distance of each pixel from the car in cm.

        Example::

            # Initialize image with the most recent depth image captured by the camera
            image = rc.camera.get_depth_image()

            # Store the distance of the object at pixel row 3, column 5
            distance = image[3][5]
        """
        pass

    @abc.abstractmethod
    def get_depth_image_async(self) -> NDArray[(480, 640), np.float32]:
        """
        Returns the current depth image without the car in "go" mode.

        Returns:
            A two dimensional array indexed from top left to the bottom right storing
            the distance of each pixel from the car in cm.

        Warning:
            This function breaks the start-update paradigm and should only be used in
            Jupyter Notebook.

        Example::

            # Initialize image with the most recent depth image captured by the camera
            image = rc.camera.get_depth_image_async()

            # Store the distance of the object at pixel row 3, column 5
            distance = image[3][5]
        """
        pass

    def get_width(self) -> int:
        """
        Returns the pixel width of the color and depth images.

        Returns:
            The width (number of pixel columns) in the color and depth images.

        Example::

            image = rc.camera.get_color_image()

            # Access the top right pixel of the image
            top_right_pixel = image[0][rc.camera.get_width() - 1]
        """
        return self._WIDTH

    def get_height(self) -> int:
        """
        Returns the pixel height of the color and depth images.

        Returns:
            The height (number of pixel rows) in the color and depth images.

        Example::

            image = rc.camera.get_color_image()

            # Access the top bottom left pixel of the image
            bottom_left_pixel = image[rc.camera.get_height() - 1][0]
        """
        return self._HEIGHT

    def get_max_range(self) -> float:
        """
        Returns the maximum distance in cm which can be detected by the depth camera.

        Returns:
            The maximum range of the depth camera.

        Example::

            depth_image = rc.camera.get_depth_image()
            center_distance = rc_utils.get_depth_image_center_distance(depth_image)

            # If center_distance is 0.0 (no data), set it to max_range
            if center_distance == 0.0:
                center_distance = rc.camera.get_max_range()
        """
        return self._MAX_RANGE
