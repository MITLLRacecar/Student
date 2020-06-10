"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Contains the Camera module of the racecar_core library
"""

import abc


class Camera(abc.ABC):
    """
    Returns the color images and depth images captured by the camera.
    """
    # The dimensions of the image in pixels
    _WIDTH = 640
    _HEIGHT = 480

    @classmethod
    @abc.abstractmethod
    def get_color_image(self):
        """
        Returns the previous color image captured by the camera.

        Returns:
            (2D numpy array of triples) A two dimensional array indexed
            from top left to the bottom right representing the pixels in the
            image. Each entry in the array is a triple of the form
            (blue, green, red) representing a single pixel.

        Note:
            Triple format = (blue, green, red), with
                blue = the amount of blue at that pixel from 0 (none) to 255 (max)
                green = the amount of green at that pixel from 0 (none) to 255 (max)
                red = the amount of red at that pixel from 0 (none) to 255 (max)

        Example:
            # Initialize image with the most recent image captured by the camera
            image = rc.camera.get_image()
        """
        pass

    @classmethod
    @abc.abstractmethod
    def get_depth_image(self):
        """
        Returns the previous depth image captured by the camera.

        Returns:
            (2D numpy array of floats) A two dimensional array indexed
            from top left to the bottom right representing the pixels in the
            image. The value of each pixel is the distance detected at that point
            in millimeters.

        Example:
            # Initialize depth_image with the most recent depth image captured
            # by the camera
            depth_image = rc.camera.get_depth_image()
        """
        pass

    @classmethod
    def get_width(self):
        """
        Returns the width of the captured images.

        Returns:
            (int) The width (number of pixel columns) of the captured images.

        Example:
            image = rc.camera.get_image()

            # Access the top right pixel of the image
            top_right_pixel = image[0, rc.camera.get_width() - 1]
        """
        return self._WIDTH

    @classmethod
    def get_height(self):
        """
        Returns the height of the captured images.

        Returns:
            (int) The height (number of pixel rows) of the captured images.

        Example:
            image = rc.camera.get_image()

            # Access the top bottom left pixel of the image
            bottom_left_pixel = image[rc.camera.get_height() - 1, 0]
        """
        return self._HEIGHT
