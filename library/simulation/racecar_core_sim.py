import struct
import socket
import sys
import select
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
    __UNITY_PORT = (__IP, 5065)
    __UNITY_ASYNC_PORT = (__IP, 5064)

    class Header(IntEnum):
        """
        The buttons on the controller
        """

        error = 0
        connect = 1
        unity_start = 2
        unity_update = 3
        unity_exit = 4
        python_finished = 5
        python_send_next = 6
        racecar_go = 7
        racecar_set_start_update = 8
        racecar_get_delta_time = 9
        racecar_set_update_slow_time = 10
        camera_get_color_image = 11
        camera_get_depth_image = 12
        camera_get_width = 13
        camera_get_height = 14
        controller_is_down = 15
        controller_was_pressed = 16
        controller_was_released = 17
        controller_get_trigger = 18
        controller_get_joystick = 19
        display_show_image = 20
        drive_set_speed_angle = 21
        drive_stop = 22
        drive_set_max_speed = 23
        lidar_get_num_samples = 24
        lidar_get_samples = 25
        physics_get_linear_acceleration = 26
        physics_get_angular_velocity = 27

    def __send_header(self, function_code: Header, isAsync: bool = False) -> None:
        self.__send_data(struct.pack("B", function_code.value), isAsync)

    def __send_data(self, data: bytes, isAsync: bool = False) -> None:
        if isAsync:
            self.__SOCKET.sendto(data, self.__UNITY_ASYNC_PORT)
        else:
            self.__SOCKET.sendto(data, self.__UNITY_PORT)

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

        self.__SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def go(self) -> None:
        print(
            ">> Python script loaded, awaiting connection from RacecarSim."
        )

        # Repeatedly try to connect to RacecarSim (async) until we receive a response
        while True:
            self.__send_header(self.Header.connect, True)
            ready = select.select([self.__SOCKET], [], [], 0.25)
            if ready[0]:
                data, _ = self.__SOCKET.recvfrom(8)
                header = int.from_bytes(data, sys.byteorder)
                if header == self.Header.connect.value:
                    print(
                        ">> Connection established with RacecarSim. Enter user program mode in RacecarSim to begin..."
                    )
                    break
                else:
                    print(">> Invalid handshake with RacecarSim, closing script...")
                    self.__send_header(self.Header.error)
                    return

        # Respond to start/update commands from RacecarSim (sync) until we receive an
        # exit or error command
        while True:
            data, _ = self.__SOCKET.recvfrom(8)
            header = int.from_bytes(data, sys.byteorder)

            if header == self.Header.unity_start.value:
                self.set_update_slow_time()
                self.__start()
            elif header == self.Header.unity_update.value:
                self.__handle_update()
            elif header == self.Header.unity_exit.value:
                print(">> Exit command received from RacecarSim, closing script...")
                break
            elif header == self.Header.error:
                print(">> Error command received from RacecarSim, closing script...")
                break
            else:
                print(f">> Error: unexpected packet with header {header} received from RacecarSim, closing script...")
                self.__send_header(self.Header.error)
                break

            self.__send_header(self.Header.python_finished)

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

    def set_update_slow_time(self, update_slow_time: float = 1.0) -> None:
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
