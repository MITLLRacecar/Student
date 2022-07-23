import sys
import numpy as np
import cv2 as cv
from nptyping import NDArray
import struct

from drone import Drone

class DroneSim(Drone):
    def __init__(self, racecar) -> None:
        self.__racecar = racecar
        self.__drone_image: NDArray[(480, 640, 3), np.uint8] = None
        self.__is_drone_image_current: bool = False

    def get_image_no_copy(self) -> NDArray[(480, 640, 3), np.uint8]:
        if not self.__is_drone_image_current:
            self.__drone_image = self.__request_drone_image(False)
            self.__is_drone_image_current = True

        return self.__drone_image

    def get_image_async(self) -> NDArray[(480, 640, 3), np.uint8]:
        return self.__request_drone_image(True)

    def get_height(self) -> float:
        self.__racecar._RacecarSim__send_header(
            self.__racecar.Header.drone_get_height
        )
        value = struct.unpack("f", self.__racecar._RacecarSim__receive_data())[0]
        return value

    def set_height(self, height: float) -> None:
        self.__racecar._RacecarSim__send_data(
            struct.pack(
                "Bf", self.__racecar.Header.drone_set_height.value, height,
            )
        )

    def return_to_car(self) -> None:
        self.__racecar._RacecarSim__send_header(
            self.__racecar.Header.drone_return_to_car
        )

    def __update(self) -> None:
        self.__is_drone_image_current = False

    def __request_drone_image(self, isAsync: bool) -> NDArray[(480, 640), np.uint8]:
        # Ask for the current color image
        self.__racecar._RacecarSim__send_header(
            self.__racecar.Header.drone_get_image, isAsync
        )

        # Read the color image as 32 packets
        raw_bytes = self.__racecar._RacecarSim__receive_fragmented(
            32, self._WIDTH * self._HEIGHT * 4, isAsync
        )
        drone_image = np.frombuffer(raw_bytes, dtype=np.uint8)
        drone_image = np.reshape(drone_image, (self._HEIGHT, self._WIDTH, 4), "C")

        drone_image = cv.cvtColor(drone_image, cv.COLOR_RGB2BGR)
        return drone_image
