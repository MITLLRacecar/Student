"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Controller module of the racecar_core library
"""

# General
import copy
from enum import Enum

# ROS
import rospy
from sensor_msgs.msg import Joy


class Controller:
    """
    Handles input from the controller and exposes constant input state
    per frame
    """

    # The ROS topic from which we read joystick information
    __TOPIC = "/joy"

    # The minimum amount that the trigger must be pressed to register a
    # non-zero value
    __TRIGGER_DEAD_ZONE = 0.05

    # The minimum amount that the joystick must be moved along an axis to
    # register a non-zero value along that axis
    __JOYSTICK_DEAD_ZONE = 0.2

    class Button(Enum):
        """
        The buttons on the controller
        """

        A = 0  # A button
        B = 1  # B button
        X = 2  # X button
        Y = 3  # Y button
        LB = 4  # Left bumper
        RB = 5  # Right bumper
        LJOY = 6  # Left joystick button
        RJOY = 7  # Right joystick button

    class Trigger(Enum):
        """
        The triggers on the controller
        """

        LEFT = 0
        RIGHT = 1

    class Joystick(Enum):
        """
        The joysticks on the controller
        """

        LEFT = 0
        RIGHT = 1

    def __init__(self, racecar):
        self.__racecar = racecar

        # Button state at the start of last frame
        self.__was_down = [False] * len(self.Button)
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

        # subscribe to the controller topic, which will call
        # __controller_callback every time the controller state changes
        self.__subscriber = rospy.Subscriber(
            self.__TOPIC, Joy, self.__controller_callback
        )

    def is_down(self, button):
        """
        Returns whether a certain button is currently pressed.

        Example:
            # This update function will print a message for every frame in which
            # the A button is held down, so multiple messages will be printed
            # if we press and hold the A button.
            def update():
                if rc.controller.is_down(rc.controller.Button.A):
                    print("The A button is currently pressed")

        Args:
            button: (Button enum) Which button to check.

        Returns:
            (bool) True if button is currently pressed.

        Note:
            The parameter must be an associated value of the Button enum,
            which is defined in the Controller module.
        """
        assert isinstance(
            button, self.Button
        ), "button must be member of the rc.controller.Button enum"

        return self.__is_down[button.value]

    def was_pressed(self, button):
        """
        Returns whether a certain button was pressed this frame.

        Example:
            # This update function will print a single message each time the A
            # button is pressed on the controller
            def update():
                if rc.controller.was_pressed(rc.controller.Button.A):
                    print("The A button was pressed")

        Args:
            button: (Button enum) Which button to check.

        Returns:
            (bool) True if button is currently pressed and was not pressed last frame.

        Note:
            The parameter must be an associated value of the Button enum,
            which is defined in the Controller module.
        """
        assert isinstance(
            button, self.Button
        ), "button must be member of the rc.controller.Button enum"

        return self.__is_down[button.value] and not self.__was_down[button.value]

    def was_released(self, button):
        """
        Returns whether a certain button was released this frame.

        Example:
            # This update function will print a single message each time the A
            # button is released on the controller
            def update():
                if rc.controller.was_pressed(rc.controller.Button.A):
                    print("The A button was released")

        Args:
            button: (Button enum) Which button to check.

        Returns:
            (bool) True if button is currently released and was pressed last frame.

        Note:
            The parameter must be an associated value of the Button enum,
            which is defined in the Controller module.
        """
        assert isinstance(
            button, self.Button
        ), "button must be member of the rc.controller.Button enum"

        return not self.__is_down[button.value] and self.__was_down[button.value]

    def get_trigger(self, trigger):
        """
        Returns the position of a certain trigger as a value from 0.0 to 1.0.

        Args:
            trigger: (Trigger enum) Which trigger to check.

        Returns:
            (float) A value from 0.0 (not pressed) to 1.0 (fully pressed).

        Note:
            The parameter must be an associated value of the Trigger enum,
            which is defined in the Controller module.

        Example:
            # Speed will receive a value from 0.0 to 1.0 based on how much the left
            # trigger is pressed
            speed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
        """
        assert isinstance(
            trigger, self.Trigger
        ), "trigger must be member of the rc.controller.Trigger enum"

        return self.__last_trigger[trigger.value]

    def get_joystick(self, joystick):
        """
        Returns the position of a certain joystick as an (x, y) tuple.

        Args:
            joystick: (Joystick enum) Which joystick to check.

        Returns:
            (float, float) The x and y coordinate of the joystick, with
            each axis ranging from -1.0 (left or down) to 1.0 (right or up).

        Note:
            The parameter must be an associated value of the Joystick enum,
            which is defined in the Controller module.

        Example:
            # x and y will be given values from -1.0 to 1.0 based on the position of
            # the left joystick
            x, y = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
        """
        assert isinstance(
            joystick, self.Joystick
        ), "joystick must be member of the rc.controller.Joystick enum"

        return self.__last_joystick[joystick.value]

    def __controller_callback(self, message):
        """
        Updates the state of Controller in response to a change in controller state.

        Args:
            message: (ROS controller message object) An object encoding the
                physical state of the controller.
        """
        self.__cur_down = [bool(b) for b in message.buttons[:6] + message.buttons[9:10]]

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
                    self.__racecar._Racecar__handle_exit()
                else:
                    self.__racecar._Racecar__handle_start()

        back = message.buttons[6]
        if back != self.__cur_back:
            self.__cur_back = back
            if back:
                if self.__cur_start:
                    self.__racecar._Racecar__handle_exit()
                else:
                    self.__racecar._Racecar__handle_back()

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
