"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Contains the Display module of the racecar_core library
"""

import abc
import numpy as np
import math
from nptyping import NDArray


class Display(abc.ABC):
    """
    Allows the user to print text and images to the RACECAR screen.
    """

    @abc.abstractmethod
    def show_color_image(self, image: NDArray[(640, 480, 3), np.uint8]) -> None:
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

    def show_depth_image(
        self, image: NDArray[(640, 480), np.float32], max_depth: int = 1000
    ) -> None:
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

        self.show_color_image(image)

    def show_lidar(
        self,
        samples: NDArray[720, np.float32],
        radius: int = 128,
        max_range: int = 1000,
    ) -> None:
        # Create a square black image with the requested radius
        image = np.zeros((2 * radius, 2 * radius, 3), np.uint8, "C")
        num_samples = len(samples)

        # Draw a red dot for each lidar sample less than max_range
        for i in range(num_samples):
            if samples[i] < max_range:
                angle = 2 * math.pi * i / num_samples
                length = radius * samples[i] / max_range
                r = int(radius - length * math.cos(angle))
                c = int(radius + length * math.sin(angle))
                image[r][c][2] = 255

        # Draw a green dot to denote the car
        for r in range(radius - 1, radius + 1):
            for c in range(radius - 1, radius + 1):
                image[r][c][1] = 255

        self.show_color_image(image)
