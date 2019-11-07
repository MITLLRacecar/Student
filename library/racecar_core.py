"""
File docstring
"""

################################################################################
# Imports
################################################################################

# General
from enum import Enum
import os # TODO: see if this can be removed
import numpy as np

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
    Class docstring
    """
    def __init__(self):
        # Modules
        self.drive = self.Drive()
        self.controller = self.Controller(self)
        
        # User provided start and update functions
        self.__user_start = None
        self.__user_update = None
        print("Racecar initialization successful")
        print("Press the START button to run your program "
            "and the BACK button to exit")

    def set_start_update(self, start, update):
        self.__user_start = start
        self.__user_update = update

    def __start(self):
        FRAMES_PER_SECOND = 60
        self.__user_start()
        timer = rospy.Rate(FRAMES_PER_SECOND)
        for i in range(300):
            self.__user_update()
            self.__update_modules()
            timer.sleep()

    def __exit(self):
        print("exit")
        exit(0)

    def __update_modules(self):
        self.drive._Drive__update()
        self.controller._Controller__update()

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

            Constants:
                MAX_SPEED = the maximum magnitude of speed allowed
                MAX_ANGLE = the maximum angle magnitude
                CONVERSION_FACTOR = the conversion factor used to convert
                    degrees to the units expected by ROS
            """
            MAX_SPEED = 5
            MAX_ANGLE = 20
            CONVERSION_FACTOR = 1 / 80

            speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
            angle = max(-MAX_ANGLE, min(MAX_ANGLE, angle))
            self.__message = AckermannDriveStamped()
            self.__message.drive.speed = speed
            self.__message.drive.steering_angle = angle # * CONVERSION_FACTOR
            print(self.__message.drive.steering_angle)

        def stop(self):
            """
            Brings the car to a stop and points the front wheels forward
            """
            self.set_speed_angle(0, 0)

        def __update(self):
            self.__publisher.publish(self.__message)

    class Controller:
        """
        Docstring
        """
        class Button(Enum):
            A = 0
            B = 1
            X = 2
            Y = 3
            LB = 4
            RB = 5
            LJOY = 6
            RJOY = 7

        class Trigger(Enum):
            LEFT = 0
            RIGHT = 1

        class Joystick(Enum):
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

            self.__was_down = [False] * len(self.Button)
            self.__is_down = [False] * len(self.Button)
            self.__cur_down = [False] * len(self.Button)

            self.__last_trigger = [0, 0]
            self.__cur_trigger = [0, 0]

            self.__last_joystick = [[0, 0], [0, 0]]
            self.__cur_joystick = [[0, 0], [0, 0]]

            # Set up callbacks
            button_map = {
                self.Button.A:self.__controller.XboxControls.A,
                self.Button.B:self.__controller.XboxControls.B,
                self.Button.X:self.__controller.XboxControls.X,
                self.Button.Y:self.__controller.XboxControls.Y,
                self.Button.LB:self.__controller.XboxControls.LB,
                self.Button.RB:self.__controller.XboxControls.RB,
                self.Button.LJOY:self.__controller.XboxControls.LEFTTHUMB,
                self.Button.RJOY:self.__controller.XboxControls.RIGHTTHUMB
            }

            trigger_map = {
                self.Trigger.LEFT:self.__controller.XboxControls.LTRIGGER,
                self.Trigger.RIGHT:self.__controller.XboxControls.RTRIGGER,
            }

            joystick_map = {
                self.Joystick.LEFT:(self.__controller.XboxControls.LTHUMBX, \
                    self.__controller.XboxControls.LTHUMBY),
                self.Joystick.RIGHT:(self.__controller.XboxControls.RTHUMBX, \
                    self.__controller.XboxControls.RTHUMBY),
            }

            for button in self.Button:
                self.__controller.setupControlCallback(
                    button_map[button],
                    lambda value : self.__button_callback(button, value)
                )

            for trigger in self.Trigger:
                self.__controller.setupControlCallback(
                    trigger_map[trigger],
                    lambda value : self.__trigger_callback(trigger, value)
                )

            for joystick in self.Joystick:
                for axis in (0, 1):
                    self.__controller.setupControlCallback(
                        joystick_map[joystick][axis],
                        lambda value : self.__joystick_callback(joystick, \
                            axis, value)
                    )

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
            if isinstance(button, self.Button):
                return self.__is_down[button.value]
            return False

        def was_pressed(self, button):
            if isinstance(button, self.Button):
                return self.__is_down[button.value] \
                    and not self.__was_down[button.value]
            return False

        def was_released(self, button):
            if isinstance(button, self.Button):
                return not self.__is_down[button.value] \
                    and self.__was_down[button.value]
            return False

        def get_trigger(self, trigger):
            if isinstance(trigger, self.Trigger):
                return self.__last_trigger[trigger.value]
            return 0

        def get_joystick(self, joystick):
            if isinstance(joystick, self.Joystick):
                return self.__last_joystick[joystick.value]
            return (0, 0)
 
        def __button_callback(self, button, value):
            self.__cur_down[button.value] = bool(value)
        
        def __trigger_callback(self, trigger, value):
            self.__cur_trigger[trigger.value] = value

        def __joystick_callback(self, joystick, axis, value):
            self.__cur_joystick[joystick.value][axis] = value

        def __start_callback(self, value):
            if value == 1:
                self.__racecar._Racecar__start()

        def __back_callback(self, value):
            if value == 1:
                self.__racecar._Racecar__exit()
                           

        def __update(self):
            self.__was_down = self.__is_down
            self.__is_down = self.__cur_down
            self.__last_trigger = self.__cur_trigger
            self.__last_joystick = self.__cur_joystick      

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

    # class Display:

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
