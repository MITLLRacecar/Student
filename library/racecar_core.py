"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Contains the Racecar class, the top level of the racecar_core library
"""

import abc


class Racecar(abc.ABC):
    """
    The top level racecar module containing several submodules which interface
    with and control the different pieces of the RACECAR hardware
    """

    def __init__(self):
        self.camera = None
        self.controller = None
        self.display = None
        self.drive = None
        self.lidar = None
        self.physics = None

    @abc.abstractmethod
    def go(self) -> None:
        """
        Starts the RACECAR, beginning in default drive mode.

        Note:
            go idles in the main thread until the car program is exited
            (START + END pressed simultaneously).
        """
        pass

    @abc.abstractmethod
    def set_start_update(self, start, update, update_slow=None) -> None:
        """
        Sets the start and update functions used in user program mode.

        Example:
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

        Args:
            start: (function) The function called once every time we enter user
                program mode.
            update: (function) The function called every frame in user program modes.

        Note:
            The provided start and update functions should not take any parameters.
        """
        pass

    @abc.abstractmethod
    def get_delta_time(self) -> float:
        """
        Returns the number of seconds elapsed in the previous frame.

        Returns:
            (float) The number of seconds between the start of the previous frame and
            the start of the current frame.

        Example:
            # Increases counter by the number of seconds elapsed in the previous frame
            counter += rc.get_delta_time()
        """
        pass

    @abc.abstractmethod
    def set_update_slow_time(self, time: float) -> None:
        """
        Changes the time between calls to update_slow.

        Args:
            time (float): The time in seconds between calls to update_slow.

        Note:
            The default value is 1 second.

        Example:
            # Sets the time between calls to update_slow to 2 seconds
            rc.set_update_slow_ratio(2)
        """
        pass
