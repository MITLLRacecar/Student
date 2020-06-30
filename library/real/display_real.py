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
    __WINDOW_NAME: str = "RACECAR display window"

    def create_window(self) -> None:
        cv.namedWindow(self.__WINDOW_NAME)

    def show_color_image(self, image: NDArray) -> None:
        cv.imshow(self.__WINDOW_NAME, image)
        cv.waitKey(1)
