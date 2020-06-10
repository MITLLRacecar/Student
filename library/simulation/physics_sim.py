import sys
import struct
import numpy as np

from physics import Physics

class PhysicsSim(Physics):
    def __init__(self, racecar):
        self.__racecar = racecar

    def get_linear_acceleration(self):
        self.__racecar._RacecarSim__send_header(
            self.__racecar.Header.physics_get_linear_acceleration
        )
        values = struct.unpack("fff", self.__racecar._RacecarSim__receive_data(12))
        return np.array(values)

    def get_angular_velocity(self):
        self.__racecar._RacecarSim__send_header(
            self.__racecar.Header.physics_get_angular_velocity
        )
        values = struct.unpack("fff", self.__racecar._RacecarSim__receive_data(12))
        return np.array(values)
