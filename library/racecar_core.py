"""
File docstring
"""

################################################################################
# Imports
################################################################################

# General
import copy
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
from ackermann_msgs.msg import AckermannDriveStamped
import XboxController


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
        print(">> Press the START button to run your program, "
            "press the BACK button to enter default drive mode, "
            "and press BACK and START at the same time to exit.")


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
        exit(0)


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
        MAX_ANGLE = 20      # The angle when the joystick is fully moved

        forwardSpeed = self.controller.get_trigger(self.controller.Trigger.LEFT)
        backSpeed = self.controller.get_trigger(self.controller.Trigger.RIGHT)
        speed = (forwardSpeed - backSpeed) * MAX_SPEED

        # If both triggers are pressed, stop for safety
        if (forwardSpeed > 0 and backSpeed > 0):
            speed = 0

        angle = self.controller.get_joystick(self.controller.Joystick.LEFT)[0] \
            * MAX_ANGLE

        self.drive.set_speed_angle(speed, angle)



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
                speed (float) = the speed, with positive for forward and
                    negative for reverse
                angle (float) = the angle of the front wheels, with positive for
                        right turns and negative for left turns
            """
            MAX_SPEED = 5   # The maximum magnitude of speed allowed
            MAX_ANGLE = 20  # The maximum angle magnitude allowed
            CONVERSION_FACTOR = 1 / 80  # converts degrees to ROS angle units

            speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
            angle = max(-MAX_ANGLE, min(MAX_ANGLE, angle))
            self.__message = AckermannDriveStamped() # TODO remove this?
            self.__message.drive.speed = speed
            self.__message.drive.steering_angle = angle # * CONVERSION_FACTOR


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
            self.__controller = XboxController.XboxController(
                controllerCallBack = None,
                joystickNo = 0,
                deadzone = 0.15,
                scale = 1,
                invertYAxis = False)

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

            self.__cur_start = 0
            self.__cur_back = 0


            # Set button callbacks
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.A,
                lambda value : self.__button_callback(self.Button.A, value)
            )
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.B,
                lambda value : self.__button_callback(self.Button.B, value)
            )
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.X,
                lambda value : self.__button_callback(self.Button.X, value)
            )
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.Y,
                lambda value : self.__button_callback(self.Button.Y, value)
            )
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.LB,
                lambda value : self.__button_callback(self.Button.LB, value)
            )
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.RB,
                lambda value : self.__button_callback(self.Button.RB, value)
            )
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.LEFTTHUMB,
                lambda value : self.__button_callback(self.Button.LJOY, value)
            )
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.RIGHTTHUMB,
                lambda value : self.__button_callback(self.Button.RJOY, value)
            )


            # Set trigger callbacks
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.LTRIGGER,
                lambda value : self.__trigger_callback(self.Trigger.LEFT, value)
            )
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.RTRIGGER,
                lambda value : self.__trigger_callback(self.Trigger.RIGHT, \
                    value)
            )


            # Set joystick callbacks
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.LTHUMBX,
                lambda value : self.__joystick_callback(self.Joystick.RIGHT, \
                    0, value)
            )
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.LTHUMBY,
                lambda value : self.__joystick_callback(self.Joystick.RIGHT, \
                    1, value)
            )
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.RTHUMBX,
                lambda value : self.__joystick_callback(self.Joystick.RIGHT, \
                    0, value)
            )
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.RTHUMBX,
                lambda value : self.__joystick_callback(self.Joystick.RIGHT, \
                    1, value)
            )


            # Set START and BACK callbacks
            self.__controller.setupControlCallback(
                self.__controller.XboxControls.START,
                self.__start_callback
            )

            self.__controller.setupControlCallback(
                self.__controller.XboxControls.BACK,
                self.__back_callback
            )

            self.__controller.start()


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
 

        def __button_callback(self, button, value):
            """
            The function called when a certain button is pressed or released

            Inputs:
                button (Button enum): which button was pressed
                value (int): 1 if pressed, 0 if released
            """
            self.__cur_down[button.value] = bool(value)


        def __trigger_callback(self, trigger, value):
            """
            The function called when a certain trigger's value changes

            Inputs:
                trigger (Trigger enum): which trigger's value changed
                value (float): how much the trigger is pressed (1.0 to 0.0)
            """
            self.__cur_trigger[trigger.value] = value


        def __joystick_callback(self, joystick, axis, value):
            """
            The function called when a certain joystick's position changes

            Inputs:
                joystick (Joystick enum): which joystick's position changed
                axis (int): either 0 for x axis or 1 for y axis
                value (float): the position of the joystick along that axis,
                    ranging from -1.0 to 1.0
            """
            self.__cur_joystick[joystick.value][axis] = value


        def __start_callback(self, value):
            """
            The function called when the START button is pressed, which either
            calls the Racecar's handle_start or handle_exit function
            """
            self.__cur_start = value
            if value:
                if self.__cur_back:
                    self.__racecar._Racecar__handle_exit()
                else:
                    self.__racecar._Racecar__handle_start()


        def __back_callback(self, value):
            """
            The function called when the BACK button is pressed, which either
            calls Racecar's handle_back or handle_exit function
            """
            self.__cur_back = value
            if value:
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
