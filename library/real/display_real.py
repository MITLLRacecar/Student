"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Display module of the racecar_core library
"""
import cv2 as cv
import os
from nptyping import NDArray

from display import Display


class DisplayReal(Display):
    __WINDOW_NAME: str = "RACECAR display window"
    __DISPLAY: str = ":1"

    def __init__(self):

        self.__display_found = (
            self.__DISPLAY
            in os.popen(
                "cd /tmp/.X11-unix && for x in X*; do echo \":${x#X}\"; done "
            ).read()
        )
        if self.__display_found:
            os.environ["DISPLAY"] = self.__DISPLAY
        else:
            print(f"Display {self.__DISPLAY} not found.")

    def create_window(self) -> None:
        if self.__display_found:
            cv.namedWindow(self.__WINDOW_NAME)
        else:
            pass

    def show_color_image(self, image: NDArray) -> None:
        if self.__display_found:
            cv.imshow(self.__WINDOW_NAME, image)
            cv.waitKey(1)
        else:
            pass
