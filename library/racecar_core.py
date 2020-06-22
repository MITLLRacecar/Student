"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Contains the Racecar class, the top level of the racecar_core library
"""

import abc
import sys
from typing import Callable, Optional

import camera
import controller
import display
import drive
import lidar
import physics


class Racecar(abc.ABC):
    """
    The top level racecar module containing several submodules which interface
    with and control the different pieces of the RACECAR hardware.
    """

    def __init__(self) -> None:
        self.camera: camera.Camera
        self.controller: controller.Controller
        self.display: display.Display
        self.drive: drive.Drive
        self.lidar: lidar.Lidar
        self.physics: physics.Physics

    @abc.abstractmethod
    def go(self) -> None:
        """
        Starts the RACECAR, beginning in default drive mode.

        Note:
            go idles blocks execution until the program is exited when START + END are
            pressed simultaneously.
        """
        pass

    @abc.abstractmethod
    def set_start_update(
        self,
        start: Callable[[], None],
        update: Callable[[], None],
        update_slow: Optional[Callable[[], None]] = None,
    ) -> None:
        """
        Sets the start and update functions used in user program mode.

        Args:
            start: A function called once when the car enters user program mode.
            update: A function called every frame in user program mode. Approximately
                60 frames occur per second.
            update_slow: A function called once per fixed time interval in user
                program mode (by default once per second).

        Note:
            The provided functions should not take any parameters.

        Example::

            # Create a racecar object
            rc = Racecar()

            # Define a start function
            def start():
                print("This function is called once")

            # Define an update function
            def update():
                print("This function is called every frame")

            # Provide the racecar with the start and update functions
            rc.set_start_update(start, update)

            # Tell the racecar to run until the program is exited
            rc.go()
        """
        pass

    @abc.abstractmethod
    def get_delta_time(self) -> float:
        """
        Returns the number of seconds elapsed in the previous frame.

        Returns:
            The number of seconds between the start of the previous frame and
            the start of the current frame.

        Example::

            # Increases counter by the number of seconds elapsed in the previous frame
            counter += rc.get_delta_time()
        """
        pass

    @abc.abstractmethod
    def set_update_slow_time(self, time: float = 1.0) -> None:
        """
        Changes the time between calls to update_slow.

        Args:
            time: The time in seconds between calls to update_slow.

        Example::

            # Sets the time between calls to update_slow to 2 seconds
            rc.set_update_slow_time(2)
        """
        pass


def create_racecar() -> Racecar:
    """
    Generates a racecar object based on the execution flags.

    Returns:
        A RacecarSim object (for use with the Unity simulation) if the program was
        executed with the -s flag, or a RacecarReal object (for use on the physical car)
        otherwise.
    """
    library_path = __file__.replace("racecar_core.py", "")

    # Create a RaceacarSim (used to interface with the Unity simulation) if the user ran
    # the program with the -s flag
    if len(sys.argv) > 1 and sys.argv[1] == "-s":
        sys.path.insert(1, library_path + "simulation")
        from racecar_core_sim import RacecarSim

        return RacecarSim()

    # Otherwise, create a RacecarReal (used to run on the physical car)
    else:
        sys.path.insert(1, library_path + "real")
        from racecar_core_real import RacecarReal

        return RacecarReal()
