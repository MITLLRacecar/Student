"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

A simple program which can be used to manually test racecar_core functionality.
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(0, "../library")
from racecar_core import Racecar
import racecar_utils as rc_utils

########################################################################################
# DO NOT MODIFY: Create Racecar Instance
########################################################################################

rc: Racecar

# Create a RaceacarSim (used to interface with the Unity simulation) if the user ran
# the program with the -s flag
if len(sys.argv) > 1 and sys.argv[1] == "-s":
    sys.path.insert(0, "../library/simulation")
    from racecar_core_sim import RacecarSim

    rc = RacecarSim()

# Otherwise, create a RacecarReal (used to run on the physical car)
else:
    sys.path.insert(0, "../library/real")
    from racecar_core_real import RacecarReal

    rc = RacecarReal()

########################################################################################
# Global variables
########################################################################################

max_speed = 0
show_triggers = False
show_joysticks = False

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global max_speed
    global show_triggers
    global show_joysticks

    print("Start function called")
    rc.set_update_slow_time(0.5)
    rc.drive.stop()

    max_speed = 0.25
    show_triggers = False
    show_joysticks = False


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global max_speed
    global show_triggers
    global show_joysticks

    # Check if each button was_pressed or was_released
    for button in rc.controller.Button:
        if rc.controller.was_pressed(button):
            print("Button {} was pressed".format(button.name))
        if rc.controller.was_released(button):
            print("Button {} was released".format(button.name))

    # Click left and right joystick to toggle showing trigger and joystick values
    left_trigger = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    right_trigger = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    left_joystick = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    right_joystick = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)

    if rc.controller.was_pressed(rc.controller.Button.LJOY):
        show_triggers = not show_triggers

    if rc.controller.was_pressed(rc.controller.Button.RJOY):
        show_joysticks = not show_joysticks

    if show_triggers:
        print("Left trigger: {}; Right trigger: {}".format(left_trigger, right_trigger))

    if show_joysticks:
        print(
            "Left joystick: {}; Right joystick: {}".format(
                left_joystick, right_joystick
            )
        )

    # Use triggers and left joystick to control car (like default drive)
    rc.drive.set_speed_angle(right_trigger - left_trigger, left_joystick[0])

    # Change max speed when the bumper is pressed
    if rc.controller.was_pressed(rc.controller.Button.LB):
        max_speed = max(1 / 16, max_speed / 2)
        rc.drive.set_max_speed(max_speed)
    if rc.controller.was_pressed(rc.controller.Button.RB):
        max_speed = min(1, max_speed * 2)
        rc.drive.set_max_speed(max_speed)

    # Capture and display color images when the A button is down
    if rc.controller.is_down(rc.controller.Button.A):
        rc.display.show_color_image(rc.camera.get_color_image())

    # Capture and display depth images when the B button is down
    elif rc.controller.is_down(rc.controller.Button.B):
        depth_image = rc.camera.get_depth_image()
        rc.display.show_depth_image(depth_image)
        print(
            "Depth center distance: {:.2f} cm".format(
                rc_utils.get_depth_image_center_distance(depth_image)
            )
        )

    # Capture and display Lidar data when the X button is down
    elif rc.controller.is_down(rc.controller.Button.X):
        lidar = rc.lidar.get_samples()
        rc.display.show_lidar(lidar)
        print(
            "Lidar forward distance: {:.2f} cm".format(
                rc_utils.get_lidar_average_distance(lidar, 0)
            )
        )

    # Show IMU data when the Y button is pressed
    if rc.controller.is_down(rc.controller.Button.Y):
        a = rc.physics.get_linear_acceleration()
        w = rc.physics.get_angular_velocity()
        print(
            "Linear acceleration: ({:5.2f},{:5.2f},{:5.2f}); ".format(a[0], a[1], a[2])
            + "Angular velocity: ({:5.2f},{:5.2f},{:5.2f})".format(w[0], w[1], w[2])
        )


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Check if each button is_down
    for button in rc.controller.Button:
        if rc.controller.is_down(button):
            print("Button {} is down".format(button.name))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
