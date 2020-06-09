import sys
import struct
import numpy as np
import binascii

from lidar import Lidar

class LidarSim(Lidar):
    __NUM_SAMPLES = 720

    def __init__(self, racecar):
        self.__racecar = racecar
        self.__ranges = None
        self.__is_current = False

    def get_num_samples(self):
        return self.__NUM_SAMPLES

    def get_samples(self):
        if not self.__is_current:
            self.__racecar._RacecarSim__send_header(self.__racecar.Header.lidar_get_samples)
            raw_bytes = self.__racecar._RacecarSim__receive_data(self.__NUM_SAMPLES * 4)
            self.__ranges = np.frombuffer(raw_bytes, dtype=np.float32)
            self.__is_current = True
        return self.__ranges

    def __update(self):
        self.__is_current = False
