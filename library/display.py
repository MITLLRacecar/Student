"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Display module of the racecar_core library
"""

import cv2 as cv

class Display:
    """
    Class docstring
    """

    def __init__(self):
        pass

    def show_image(self, image):
        cv.imshow("display window", image)
        cv.waitKey(1)

    def show_text(self, text, size, color):
        pass
