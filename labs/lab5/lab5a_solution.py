"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 5A - LIDAR Safety Stop
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
MIN_STOP_DISTANCE = 40  # cm
MAX_STOP_DISTANCE = 100  # cm
ALPHA = 0.2  # Amount to use current_speed when updating forward_speed

# The (min, max) degrees to consider when measuring forward and rear distances
FRONT_WINDOW = (-10, 10)
REAR_WINDOW = (170, 190)

# Amount to increase stop distance (cm) per speed (cm/s) squared
STOP_DISTANCE_SCALE = 40.0 / 10000

# slow_distance / stop_distance
SLOW_DISTANCE_RATIO = 1.5

# >> Variables
cur_speed = 0  # cm/s
prev_forward_dist = 0  # cm
prev_back_dist = 0  # cm

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global cur_speed
    global prev_forward_dist
    global prev_back_dist

    # Have the car begin at a stop
    rc.drive.stop()

    # Initialize variables
    cur_speed = 0
    scan = rc.lidar.get_samples()
    _, forward_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)
    _, back_dist = rc_utils.get_lidar_closest_point(scan, REAR_WINDOW)

    # Print start message
    print(
        ">> Lab 5A - LIDAR Safety Stop\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Right bumper = override forward safety stop\n"
        "   Left trigger = accelerate backward\n"
        "   Left bumper = override rear safety stop\n"
        "   Left joystick = turn front wheels\n"
        "   A button = print current speed and angle\n"
        "   B button = print forward and back distances"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global cur_speed
    global prev_forward_dist
    global prev_back_dist

    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    # Calculate the distance in front of and behind the car
    scan = rc.lidar.get_samples()
    _, forward_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)
    _, back_dist = rc_utils.get_lidar_closest_point(scan, REAR_WINDOW)

    frame_speed: float
    if forward_dist < back_dist:
        frame_speed = (prev_forward_dist - forward_dist) / rc.get_delta_time()
    else:
        frame_speed = -(prev_back_dist - back_dist) / rc.get_delta_time()

    cur_speed += ALPHA * (frame_speed - cur_speed)
    prev_forward_dist = forward_dist
    prev_back_dist = back_dist
    stop_distance: float

    # FORARD SAFETY STOP: We are about to hit something in front of us
    if not rc.controller.is_down(rc.controller.Button.RB) and cur_speed > 0:
        # Calculate slow and stop distances based on the current speed
        stop_distance = rc_utils.clamp(
            MIN_STOP_DISTANCE + cur_speed * abs(cur_speed) * STOP_DISTANCE_SCALE,
            MIN_STOP_DISTANCE,
            MAX_STOP_DISTANCE,
        )
        slow_distance = stop_distance * SLOW_DISTANCE_RATIO

        # If we are past slow_distance, reduce speed proportional to how close we are
        # to stop_distance
        if stop_distance < forward_dist < slow_distance:
            speed = min(
                speed,
                rc_utils.remap_range(
                    forward_dist, stop_distance, slow_distance, 0, 0.5
                ),
            )
            print("Forward safety slow: speed limited to {}".format(speed))

        # Safety stop if we are passed stop_distance by reversing at a speed
        # proportional to how far we are past stop_distance
        if 0 < forward_dist < stop_distance:
            speed = rc_utils.remap_range(
                forward_dist, 0, stop_distance, -4, -0.2
            )
            speed = rc_utils.clamp(speed, -1, -0.2)
            print("Forward safety stop: reversing at {}".format(speed))

    # REAR SAFETY STOP: We are about to hit something behind us
    if not rc.controller.is_down(rc.controller.Button.LB) and cur_speed < 0:
        stop_distance = rc_utils.clamp(
            MIN_STOP_DISTANCE - cur_speed * abs(cur_speed) * STOP_DISTANCE_SCALE,
            MIN_STOP_DISTANCE,
            MAX_STOP_DISTANCE,
        )
        slow_distance = stop_distance * SLOW_DISTANCE_RATIO

        if stop_distance < back_dist < slow_distance:
            speed = max(
                speed,
                rc_utils.remap_range(back_dist, stop_distance, slow_distance, 0, -0.5),
            )
            print("Back safety slow: speed limited to {}".format(speed))

        if 0 < back_dist < stop_distance:
            speed = rc_utils.remap_range(
                back_dist, 0, stop_distance, 4, 0.2, True
            )
            speed = rc_utils.clamp(speed, 0.2, 1)
            print("Back safety stop: accelerating forward at {}".format(speed))

    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the distance of the closest object in front of and behind the car
    if rc.controller.is_down(rc.controller.Button.B):
        print("Forward distance:", forward_dist, "Back distance:", back_dist)

    # Print cur_speed estimate and stop distance when the X button is held down
    if rc.controller.is_down(rc.controller.Button.X):
        print(
            "Current speed estimate: {:.2f} cm/s, Stop distance: {:.2f}".format(
                cur_speed, stop_distance
            )
        )

    # Display the current LIDAR scan
    rc.display.show_lidar(scan)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
