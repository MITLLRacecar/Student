import cv2 as cv

from display import Display


class DisplaySim(Display):
    __WINDOW_NAME = "RacecarSim display window"

    def show_image(self, image):
        cv.namedWindow(self.__WINDOW_NAME, cv.WINDOW_NORMAL)
        cv.imshow(self.__WINDOW_NAME, image)
        cv.waitKey(1)  # TODO is this line necessary?
