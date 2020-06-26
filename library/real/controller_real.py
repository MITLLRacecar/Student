"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Controller module of the racecar_core library
"""

from controller import Controller

# General
import copy
from typing import Tuple

# ROS2
import rclpy as ros2
from sensor_msgs.msg import Joy


class ControllerReal(Controller):
    # The ROS topic from which we read joystick information
    __TOPIC = "/joy"

    # The minimum amount that the trigger must be pressed to register a
    # non-zero value
    __TRIGGER_DEAD_ZONE = 0.05

    # The minimum amount that the joystick must be moved along an axis to
    # register a non-zero value along that axis
    __JOYSTICK_DEAD_ZONE = 0.2

    def __init__(self, racecar):
        self.__racecar = racecar
        # print(f"Length of self.Button: {len(self.Button)}")
        # Button state at the start of last frame
        self.__was_down = [False] * len(self.Button)
        # print(f"Length of __was_down: {len(self.__was_down)}")
        # Button state at the start of this frame
        self.__is_down = [False] * len(self.Button)
        # Button state received since the start of this frame
        self.__cur_down = [False] * len(self.Button)

        # Trigger state at the start of this frame
        self.__last_trigger = [0, 0]
        # Trigger state received since the start of this frame
        self.__cur_trigger = [0, 0]

        # Joystick state at the start of this frame
        self.__last_joystick = [[0, 0], [0, 0]]
        # Joystick state received since the start of this frame
        self.__cur_joystick = [[0, 0], [0, 0]]

        # Current start and back button state
        self.__cur_start = 0
        self.__cur_back = 0

        # ROS node
        self.node = ros2.create_node("controller")

        # subscribe to the controller topic, which will call
        # __controller_callback every time the controller state changes
        self.__subscriber = self.node.create_subscription(
            Joy, self.__TOPIC, self.__controller_callback, 1
        )

    def is_down(self, button: Controller.Button) -> bool:
        return self.__is_down[button.value]

    def was_pressed(self, button: Controller.Button) -> bool:
        return self.__is_down[button.value] and not self.__was_down[button.value]

    def was_released(self, button: Controller.Button) -> bool:
        return not self.__is_down[button.value] and self.__was_down[button.value]

    def get_trigger(self, trigger: Controller.Trigger) -> float:
        return self.__last_trigger[trigger.value]

    def get_joystick(self, joystick: Controller.Joystick) -> Tuple[float, float]:
        return self.__last_joystick[joystick.value]

    def __controller_callback(self, message):
        """
        Updates the state of Controller in response to a change in controller state.

        Args:
            message: (ROS controller message object) An object encoding the
                physical state of the controller.
        """
        self.__cur_down = [bool(b) for b in message.buttons[:6] + message.buttons[9:11]]

        self.__cur_trigger = [
            self.__convert_trigger_value(message.axes[2]),
            self.__convert_trigger_value(message.axes[5]),
        ]

        self.__cur_joystick = [
            (
                self.__convert_joystick_value(message.axes[0]),
                self.__convert_joystick_value(message.axes[1]),
            ),
            (
                self.__convert_joystick_value(message.axes[3]),
                self.__convert_joystick_value(message.axes[4]),
            ),
        ]

        start = message.buttons[7]
        if start != self.__cur_start:
            self.__cur_start = start
            if start:
                if self.__cur_back:
                    self.__racecar._RacecarReal__handle_exit()
                else:
                    self.__racecar._RacecarReal__handle_start()

        back = message.buttons[6]
        if back != self.__cur_back:
            self.__cur_back = back
            if back:
                if self.__cur_start:
                    self.__racecar._RacecarReal__handle_exit()
                else:
                    self.__racecar._RacecarReal__handle_back()

    def __update(self):
        """
        Updates the input registers when the current frame ends.
        """
        self.__was_down = copy.deepcopy(self.__is_down)
        self.__is_down = copy.deepcopy(self.__cur_down)
        self.__last_trigger = copy.deepcopy(self.__cur_trigger)
        self.__last_joystick = copy.deepcopy(self.__cur_joystick)

    def __convert_trigger_value(self, value):
        """
        Converts a received trigger value into the desired range.

        Args:
            value: (float) The value of the controller provided in the ROS message.
        """
        value = (1.0 - value) / 2
        if value < self.__TRIGGER_DEAD_ZONE:
            return 0
        return value

    def __convert_joystick_value(self, value):
        """
        Converts a received joystick axis value into the desired range.

        Args:
            value: (float) The value of the joystick axis provided in the ROS message.
        """
        if abs(value) < self.__JOYSTICK_DEAD_ZONE:
            return 0
        return value
