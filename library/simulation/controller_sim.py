import sys
import struct
from enum import IntEnum

from controller import Controller


class ControllerSim(Controller):
    def __init__(self, racecar):
        self.__racecar = racecar

    def is_down(self, button):
        self.__racecar._RacecarSim__send_data(
            struct.pack(
                "BB", self.__racecar.Header.controller_is_down.value, button.value
            )
        )
        return bool.from_bytes(self.__racecar._RacecarSim__receive_data(), sys.byteorder)

    def was_pressed(self, button):
        self.__racecar._RacecarSim__send_data(
            struct.pack(
                "BB", self.__racecar.Header.controller_was_pressed.value, button.value,
            )
        )
        return bool.from_bytes(self.__racecar._RacecarSim__receive_data(), sys.byteorder)

    def was_released(self, button):
        self.__racecar._RacecarSim__send_data(
            struct.pack(
                "BB", self.__racecar.Header.controller_was_pressed.value, button.value,
            )
        )
        return bool.from_bytes(self.__racecar._RacecarSim__receive_data(), sys.byteorder)

    def get_trigger(self, trigger):
        self.__racecar._RacecarSim__send_data(
            struct.pack(
                "BB", self.__racecar.Header.controller_get_trigger.value, trigger.value,
            )
        )
        [value] = struct.unpack("f", self.__racecar._RacecarSim__receive_data())
        return value

    def get_joystick(self, joystick):
        self.__racecar._RacecarSim__send_data(
            struct.pack(
                "BB",
                self.__racecar.Header.controller_get_joystick.value,
                joystick.value,
            )
        )
        return struct.unpack("ff", self.__racecar._RacecarSim__receive_data(8))
