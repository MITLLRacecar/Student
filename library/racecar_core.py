"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Racecar class, the top level of the racecar_core library
"""

# General
from datetime import datetime
import time # TODO: can we remove this?
import threading
import os # TODO: see if this can be removed

# ROS
import rospy
from sensor_msgs.msg import LaserScan

# racecar_core modules
import drive
import controller


################################################################################
# Racecar class
################################################################################

class Racecar:
    """
    The top level racecar module containing several submodules which interface 
    with and control the different pieces of the RACECAR hardware
    """

    def __init__(self):
        # Modules
        self.drive = self.Drive()
        self.controller = self.Controller(self)
        self.display = self.Display()
        self.camera = self.Camera()

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

        Example: See set_start_update below
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

        Note: The provided start and update functions should not take any
        parameters

        Example:
        ```Python
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
        ```
        """
        self.__user_start = start
        self.__user_update = update


    def get_delta_time(self):
        """
        Returns the number of seconds elapsed in the previous frame

        Output (float): The number of seconds between the start of the previous
            frame and the start of the current frame

        Example:
        ```Python
        # Increases counter by the number of seconds elapsed in the previous
        # frame.
        counter += rc.get_delta_time()
        ```
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
        pass
    

    def __default_update(self):
        """
        The update function for default drive mode

        Controls:
            Left trigger: Accelerate forward
            Right trigger: Accelerate backward
            Left joystick: Turn left and right
            A button: Print "Kachow!" to the terminal
        """
        MAX_SPEED = 1.0     # The speed when the trigger is fully pressed
        MAX_ANGLE = 1.0     # The angle when the joystick is fully moved

        forwardSpeed = self.controller.get_trigger(\
            self.controller.Trigger.RIGHT)
        backSpeed = self.controller.get_trigger(self.controller.Trigger.LEFT)
        speed = (forwardSpeed - backSpeed) * MAX_SPEED

        # If both triggers are pressed, stop for safety
        if forwardSpeed > 0 and backSpeed > 0:
            speed = 0

        angle = self.controller.get_joystick(self.controller.Joystick.LEFT)[0] \
            * MAX_ANGLE

        self.drive.set_speed_angle(speed, angle)

        if self.controller.was_pressed(self.controller.Button.A):
            print("Kachow!")


    class Display:
        """
        Class docstring
        """

        def __init__(self):
            pass

        def show_image(self, image):
            cv.imshow("display", image)

        def show_text(self, text, size, color):
            pass
