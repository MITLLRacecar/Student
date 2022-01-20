import sys
import struct
import numpy as np
from nptyping import NDArray

from physics import Physics

class PhysicsSim(Physics):
    def __init__(self, racecar) -> None:
        self.__racecar = racecar

    def get_linear_acceleration(self) -> NDArray[3, np.float32]:
        self.__racecar._RacecarSim__send_header(
            self.__racecar.Header.physics_get_linear_acceleration
        )
        values = struct.unpack("fff", self.__racecar._RacecarSim__receive_data(12))
        return np.array(values)

    def get_angular_velocity(self) -> NDArray[3, np.float32]:
        self.__racecar._RacecarSim__send_header(
            self.__racecar.Header.physics_get_angular_velocity
        )
        values = struct.unpack("fff", self.__racecar._RacecarSim__receive_data(12))
        return np.array(values)

    #Added this get_position() function - should return an array of size 3 with the x,y,z coordinates
    def get_position(self) -> NDArray[3, np.float32]:
        self.__racecar._RacecarSim__send_header(
            self.__racecar.Header.physics_get_position
        )
        values = struct.unpack("fff", self.__racecar._RacecarSim__receive_data(12))
        return np.array(values)
