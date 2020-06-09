"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Contains the Display module of the racecar_core library
"""

import abc
import numpy as np


class Display:
    """
    Allows the user to print text and images to the RACECAR screen.
    """

    @classmethod
    @abc.abstractmethod
    def show_color_image(self, image):
        """
        Displays an image on a window of the RACECAR desktop.

        Args:
            image: (2D numpy array of triples) The image to display to the
                screen encoded as a 2D array of pixels, where each pixel is
                stored as a (blue, green, red) triple.

        Example:
            image = rc.camera.get_image();

            # Show the image captured by the camera
            rc.display.show_image(image);
        """
        pass

    @classmethod
    def show_depth_image(self, image, max_depth = 1000):
        """
        Displays an image on a window of the RACECAR desktop.

        Args:
            image: (2D numpy array of triples) The image to display to the
                screen encoded as a 2D array of pixels, where each pixel is
                stored as a (blue, green, red) triple.

        Example:
            image = rc.camera.get_image();

            # Show the image captured by the camera
            rc.display.show_image(image);
        """
        # Clip anything above max_depth
        np.clip(image, None, max_depth, image)

        # Shift down 1 unit so that 0 (no data) becomes the "farthest" color
        image = (image - 1) % max_depth

        # Scale measurements so that closest depth becomes 255 (white) and max_depth
        # becomes 0 (black).
        image = 1 - (image / max_depth)

        self.show_color_image(self, image)
