"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Display module of the racecar_core library
"""
import cv2 as cv
from nptyping import NDArray

from display import Display


class DisplayReal(Display):
    def show_color_image(self, image: NDArray) -> None:
        cv.imshow("RACECAR display window", image)
        cv.waitKey(1)
