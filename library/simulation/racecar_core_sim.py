import math
import struct
import socket
import time
import sys
from enum import IntEnum
from typing import Callable, Optional

import camera_sim
import controller_sim
import display_sim
import drive_sim
import lidar_sim
import physics_sim

from racecar_core import Racecar


class RacecarSim(Racecar):
    __IP = "127.0.0.1"
    __UNITY_PORT = 5065
    __PYTHON_PORT = 5066
    __SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    class Header(IntEnum):
        """
        The buttons on the controller
        """

        error = 0
        unity_start = 1
        unity_update = 2
        unity_exit = 3
        python_finished = 4
        python_send_next = 5
        racecar_go = 6
        racecar_set_start_update = 7
        racecar_get_delta_time = 8
        racecar_set_update_slow_time = 9
        camera_get_color_image = 10
        camera_get_depth_image = 11
        camera_get_width = 12
        camera_get_height = 13
        controller_is_down = 14
        controller_was_pressed = 15
        controller_was_released = 16
        controller_get_trigger = 17
        controller_get_joystick = 18
        display_show_image = 19
        drive_set_speed_angle = 20
        drive_stop = 21
        drive_set_max_speed = 22
        lidar_get_num_samples = 23
        lidar_get_samples = 24
        physics_get_linear_acceleration = 25
        physics_get_angular_velocity = 26

    def __send_header(self, function_code: Header) -> None:
        self.__send_data(struct.pack("B", function_code.value))

    def __send_data(self, data: bytes) -> None:
        self.__SOCKET.sendto(data, (self.__IP, self.__UNITY_PORT))

    def __receive_data(self, buffer_size: int = 8) -> bytes:
        data, _ = self.__SOCKET.recvfrom(buffer_size)
        return data

    def __init__(self) -> None:
        self.camera = camera_sim.CameraSim(self)
        self.controller = controller_sim.ControllerSim(self)
        self.display = display_sim.DisplaySim()
        self.drive = drive_sim.DriveSim(self)
        self.physics = physics_sim.PhysicsSim(self)
        self.lidar = lidar_sim.LidarSim(self)

        self.__start: Callable[[], None]
        self.__update: Callable[[], None]
        self.__update_slow: Optional[Callable[[], None]]
        self.__update_slow_time: float = 1
        self.__update_slow_counter: float = 0
        self.__delta_time: float = -1

        self.__SOCKET.bind((self.__IP, self.__PYTHON_PORT))

    def go(self) -> None:
        print(">> Python script loaded, please enter user program mode in Unity")
        while True:
            data, _ = self.__SOCKET.recvfrom(8)
            header = int.from_bytes(data, sys.byteorder)

            response = self.Header.error
            if header == self.Header.unity_start.value:
                self.__start()
                response = self.Header.python_finished
            elif header == self.Header.unity_update.value:
                self.__handle_update()
                response = self.Header.python_finished
            elif header == self.Header.unity_exit.value:
                print(">> Exit command received from Unity")
                break
            else:
                print(">> Error: unexpected packet from Unity", header)

            self.__send_header(response)

    def set_start_update(
        self,
        start: Callable[[], None],
        update: Callable[[], None],
        update_slow: Optional[Callable[[], None]] = None,
    ) -> None:
        self.__start = start
        self.__update = update
        self.__update_slow = update_slow

    def get_delta_time(self) -> float:
        if self.__delta_time < 0:
            self.__send_header(self.Header.racecar_get_delta_time)
            [value] = struct.unpack("f", self.__receive_data())
            self.__delta_time = value
        return self.__delta_time

    def set_update_slow_time(self, update_slow_time: float) -> None:
        self.__update_slow_time = update_slow_time

    def __handle_update(self) -> None:
        self.__update()

        self.__delta_time = -1
        if self.__update_slow is not None:
            self.__update_slow_counter -= self.get_delta_time()
            if self.__update_slow_counter < 0:
                self.__update_slow()
                self.__update_slow_counter = self.__update_slow_time

        self.camera._CameraSim__update()
        self.controller._ControllerSim__update()
        self.lidar._LidarSim__update()
