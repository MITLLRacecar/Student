"""
File docstring
"""

################################################################################
# Imports
################################################################################

# General
import copy
import sys
from datetime import datetime, timedelta
import time # TODO: can we remove this?
import threading
from enum import Enum
import os # TODO: see if this can be removed
import cv2

# ROS
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped


################################################################################
# Racecar class
################################################################################

class Racecar:
    """
    The core racecar module which contains several submodules which interface 
    with and control the different pieces of hardware
    """

    def __init__(self):
        # Modules
        self.drive = self.Drive()
        self.controller = self.Controller(self)
        self.display = self.Display()
        
        # User provided start and update functions
        self.__user_start = None
        self.__user_update = None

        # True if the main thread should be running
        self.__running = False

        # Variables relating to the run thread
        self.__run_thread = None
        self.__cur_update = self.__default_update
        self.__cur_frame_time = datetime.now()
        self.__last_frame_time = datetime.now()

        # Start run_thread in default drive mode
        self.__handle_back()
        self.__run_thread = threading.Thread(target=self.__run)
        self.__run_thread.daemon = True
        self.__run_thread.start()

        # Print welcome message
        print(">> Racecar initialization successful")
        print(">> Controlls:\n"
            "     START button = run your program\n"
            "     BACK button = enter default drive mode\n"
            "     BACK + START buttons simultaneously = exit the program\n"
            "     CTRL + Z on keyboard = force quit the program")


    def go(self):
        """
        Idles in the main thread until the car program is exited 
        (START + END pressed simultaneously)
        """
        self.__running = True
        while(self.__running):
            pass


    def set_start_update(self, start, update):
        """
        Sets the start and update functions used in user program mode

        Inputs:
            start (function): The function called once every time we enter 
                user program mode
            update (function): The function called every frame in user program
                modes
        """
        self.__user_start = start
        self.__user_update = update


    def get_delta_time(self):
        """
        Returns the number of seconds elapsed in the previous frame

        Output (float): The number of seconds between the start of the previous
            frame and the start of the current frame
        """
        return (self.__cur_frame_time - self.__last_frame_time).total_seconds()


    def __handle_start(self):
        """
        Handles when the START button is pressed by entering user program mode
        """
        if self.__user_start is None or self.__user_update is None:
            print(">> No user start and update functions found.  "
                "Did you call set_start_update with valid start and "
                "update functions?")
        else:
            print(">> Entering user program mode")
            self.__user_start()
            self.__cur_update = self.__user_update


    def __handle_back(self):
        """
        Handles when the BACK button is pressed by entering default drive mode
        """
        print(">> Entering default drive mode")
        self.__default_start()
        self.__cur_update = self.__default_update


    def __handle_exit(self):
        """
        Handles when BACK and START are pressed together by exiting the program
        """
        print(">> Goodbye!")
        self.__running = False


    def __run(self):
        """
        Calls the current update function (determined by the current mode) 
        and update_modules functions once per frame
        """
        FRAME_RATE = 60     # Number of frames per second

        timer = rospy.Rate(FRAME_RATE)
        while True:
            self.__last_frame_time = self.__cur_frame_time #TODO: deep copy?
            self.__cur_frame_time = datetime.now()
            self.__cur_update()
            self.__update_modules()
            timer.sleep()


    def __update_modules(self):
        """
        Calls the update function on each module
        """
        self.drive._Drive__update()
        self.controller._Controller__update()


    def __default_start(self):
        """
        The start function for default drive mode
        """
        image = cv2.imread('~/img/car.png')
        if image is None:
            print("Image is none")
        else:
            self.display.show_image(image)


    def __default_update(self):
        """
        The update function for default drive mode, which controls the car with
        the triggers and left joystick
        """
        MAX_SPEED = 1.0     # The speed when the trigger is fully pressed
        MAX_ANGLE = 1.0     # The angle when the joystick is fully moved

        forwardSpeed = self.controller.get_trigger(self.controller.Trigger.LEFT)
        backSpeed = self.controller.get_trigger(self.controller.Trigger.RIGHT)
        speed = (forwardSpeed - backSpeed) * MAX_SPEED

        print("forwardSpeed", forwardSpeed)
        print("backSpeed", backSpeed)

        # If both triggers are pressed, stop for safety
        if forwardSpeed > 0 and backSpeed > 0:
            speed = 0

        angle = self.controller.get_joystick(self.controller.Joystick.LEFT)[0] \
            * MAX_ANGLE

        self.drive.set_speed_angle(speed, angle)

        if self.controller.was_pressed(self.controller.Button.A):
            print("Kachow!")



    class Drive:
        """
        Controls the car's movement by allowing the user to set the state
        associated with speed and turning
        """
        __TOPIC = "/drive"


        def __init__(self):
            self.__publisher = rospy.Publisher(self.__TOPIC, \
                AckermannDriveStamped, queue_size=1)
            self.__message = AckermannDriveStamped()


        def set_speed_angle(self, speed, angle):
            """
            Sets the speed at which the wheels turn and the angle of the front
            wheels

            Inputs:
                speed (float) = the speed from -1 to 1, with positive for 
                    forward and negative for reverse
                angle (float) = the angle of the front wheels from -1 to 1, 
                        with positive for right negative for left
            """
            MAX_SPEED = 1  # The maximum speed magnitude allowed
            MAX_ANGLE = 1  # The maximum angle magnitude allowed
            SPEED_CONVERSION_FACTOR = 4
            ANGLE_CONVERSION_FACTOR = 1 / 4.0

            print(speed)

            speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
            angle = max(-MAX_ANGLE, min(MAX_ANGLE, angle))
            self.__message.drive.speed = speed * SPEED_CONVERSION_FACTOR
            self.__message.drive.steering_angle = angle * \
                ANGLE_CONVERSION_FACTOR


        def stop(self):
            """
            Brings the car to a stop and points the front wheels forward
            """
            self.set_speed_angle(0, 0)


        def __update(self):
            """
            Publishes the current drive message
            """
            self.__publisher.publish(self.__message)



    class Controller:
        """
        Handles input from the controller and exposes constant input state
        per frame
        """
        TRIGGER_DEAD_ZONE = 0.05
        JOYSTICK_DEAD_ZONE = 0.2

        class Button(Enum):
            """
            The buttons on the controller
            """
            A = 0       # A button
            B = 1       # B button
            X = 2       # X button
            Y = 3       # Y button
            LB = 4      # Left bumper
            RB = 5      # Right bumper
            LJOY = 6    # Left joystick button
            RJOY = 7    # Right joystick button


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
            self.__subscriber = rospy.Subscriber('/joy', 
                Joy, self.__controller_callback)


        def is_down(self, button):
            """
            Returns whether a certain button is currently pressed

            Inputs:
                button (Button enum) = which button to check

            Output (bool): True if button is currently pressed
            """
            if isinstance(button, self.Button):
                return self.__is_down[button.value]
            return False


        def was_pressed(self, button):
            """
            Returns whether a certain button was pressed this frame

            Inputs:
                button (Button enum) = which button to check

            Output (bool): True if button is currently pressed and was not pressed
                last frame
            """
            if isinstance(button, self.Button):
                return self.__is_down[button.value] \
                    and not self.__was_down[button.value]
            return False


        def was_released(self, button):
            """
            Returns whether a certain button was released this frame

            Inputs:
                button (Button enum) = which button to check

            Output (bool): True if button is currently released and was pressed 
                last frame
            """
            if isinstance(button, self.Button):
                return not self.__is_down[button.value] \
                    and self.__was_down[button.value]
            return False


        def get_trigger(self, trigger):
            """
            Returns the position of a certain trigger

            Inputs:
                trigger (Trigger enum) = which trigger to check

            Output (float): A value from 0.0 (not pressed) to 
                1.0 (fully pressed)
            """
            if isinstance(trigger, self.Trigger):
                return self.__last_trigger[trigger.value]
            return 0


        def get_joystick(self, joystick):
            """
            Returns the position of a certain joystick

            Inputs:
                joystick (Joystick enum) = which joystick to check

            Output (float, float): The x and y coordinate of the joystick, with
                each axis ranging from -1.0 to 1.0
            """
            if isinstance(joystick, self.Joystick):
                return self.__last_joystick[joystick.value]
            return (0, 0)


        def __controller_callback(self, msg):
            """
            TODO: Docstring
            """            
            self.__cur_down = [bool(b) \
                for b in msg.buttons[:6] + msg.buttons[9:10]]

            self.__cur_trigger = 
                [self.__convert_trigger_value(msg.axes[2]), \
                 self.__convert_trigger_value(msg.axes[5])]

            self.__cur_joystick = 
                [(self.__convert_joystick_value(msg.axes[0]), 
                  self.__convert_joystick_value(msg.axes[1])),
                 (self.__convert_joystick_value(msg.axes[3]),
                  self.__convert_joystick_value(msg.axes[4]))]

            start = msg.buttons[7]
            if start != self.__cur_start:
                self.__cur_start = start
                if start:
                    if self.__cur_back:
                        self.__racecar._Racecar__handle_exit()
                    else:
                        self.__racecar._Racecar__handle_start()

            back = msg.buttons[6]
            if back != self.__cur_back:
                self.__cur_back = back
                if back:
                    if self.__cur_start:
                        self.__racecar._Racecar__handle_exit()
                    else:
                        self.__racecar._Racecar__handle_back()


        def __update(self):
            """
            Updates the stored input registers when the current frame ends
            """
            self.__was_down = copy.deepcopy(self.__is_down)
            self.__is_down = copy.deepcopy(self.__cur_down)
            self.__last_trigger = copy.deepcopy(self.__cur_trigger)
            self.__last_joystick = copy.deepcopy(self.__cur_joystick)

        def __convert_trigger_value(self, value):
            """
            TODO: docstring 
            """
            value = (value + 1.0) / 2
            if value < self.TRIGGER_DEAD_ZONE:
                return 0
            return value

        def __convert_joystick_value(self, value):
            """
            TODO: docstring
            """
            if value < self.JOYSTICK_DEAD_ZONE:
                return 0
            return value



    # class Physics:
    #     def get_acceleration(self) -> Tuple[float, float, float]:
    #         return (0, 0, 0)

    #     def get_angular_velocity(self) -> Tuple[float, float, float]:
    #         return (0, 0, 0)

    #     def get_speed(self) -> float:
    #         return 0

    class Camera:
        """
        Provides the user with basic image capture functionality so they can get rbg/depth information from
        the Camera on the robot.
        """

        __TOPIC = "/camera"

        def __init__(self):
            self.__subscriber = rospy.Subscriber(self.__TOPIC, Image, callback=self.__image_callback)
            self.__last_image = None

        def __image_callback(self, msg):
            self.__last_image = msg.data

        def get_image(self):
            """
            Get's color image data from Intel Realsense Camera and returns the raw data in numpy array
            """
            return np.fromstring(self.__last_image,dtype=np.uint8).reshape((480,-1,3))[...,::-1]

        def get_depth_image(self):
            """
            Get's color+depth image (4 channels) from Intel Realsense Camera and returns the raw data in numpy array
            """
            #TODO Add depth channel to image gotten from realsense
            return None

    # class Lidar:
    #     def get_raw(self):
    #         return None
        
    #     def get_map(self):
    #         return None    

    # class GPIO:
    #     def get_pin(self, pin: int) -> int:
    #         return 0

    #     def set_pin(self, pin: int, value: int):
    #         raise NotImplementedError()


    # class Controller:
    #     def is_pressed(self, button) -> bool:
    #         return False

    #     def get_trigger(self, trigger) -> float:
    #         return 0

    #     def get_joystick(self, joystick) -> Tuple[float, float]:
    #         return (0, 0)

    class Display:
        """
        Class docstring
        """

        def __init__(self):
            pass

        def show_image(self, image):
            cv2.imshow("display", image)

        def show_text(self, text, size, color):
            pass



    # class Sound: 


        

    # def image_callback(self, msg):
    #     self.last_image = msg.data
        
    # def scan_callback(self, msg):
    #     self.last_scan = msg.ranges
        
    # def get_lidar(self):
    #     return None



    # def show_image(self, image):
    #     raise NotImplementedError()

    # def show_text(self, text: str, size: float = 12, color: Tuple[float, float, float] = (0, 0, 0)):
    #     raise NotImplementedError()
    
    # def get_mic_amplitude(self, ) -> float:
    #     return 0

    # def set_wav(self, wav):
    #     raise NotImplementedError()

    # def play_wave(self):
    #     raise NotImplementedError()

    # def is_wav_playing(self) -> bool:
    #     return False
