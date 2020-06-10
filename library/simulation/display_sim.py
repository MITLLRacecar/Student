import cv2 as cv
import numpy as np
from nptyping import NDArray

from display import Display


class DisplaySim(Display):
    __WINDOW_NAME: str = "RacecarSim display window"

    def show_color_image(self, image: NDArray[(480, 640, 3), np.uint8]) -> None:
        cv.namedWindow(self.__WINDOW_NAME, cv.WINDOW_NORMAL)
        cv.imshow(self.__WINDOW_NAME, image)
        cv.waitKey(1)
