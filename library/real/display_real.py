"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Display module of the racecar_core library
"""

from display import Display
import cv2 as cv


class DisplayReal(Display):
    def show_image(self, image):
        cv.imshow("RACECAR display window", image)
        cv.waitKey(1)
