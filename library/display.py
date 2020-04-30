"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Display module of the racecar_core library
"""

import cv2 as cv


class Display:
    """
    Allows the user to print text and images to the RACECAR screen.
    """

    def __init__(self):
        pass

    def show_image(self, image):
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
        cv.imshow("display window", image)
        cv.waitKey(1)
