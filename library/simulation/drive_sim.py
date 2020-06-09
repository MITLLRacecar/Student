import struct

from drive import Drive

class DriveSim(Drive):
    def __init__(self, racecar):
        self.__racecar = racecar

    def set_speed_angle(self, speed, angle):
        self.__racecar._RacecarSim__send_data(
            struct.pack(
                "Bff",
                self.__racecar.Header.drive_set_speed_angle.value,
                speed,
                angle,
            )
        )

    def stop(self):
        self.__racecar._RacecarSim__send_header(self.__racecar.Header.drive_stop)

    def set_max_speed(self, max_speed):
        self.__racecar._RacecarSim__send_data(
            struct.pack(
                "Bf",
                self.__racecar.Header.drive_set_max_speed.value,
                max_speed,
            )
        )
