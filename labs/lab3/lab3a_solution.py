"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3A - Depth Camera Safety Stop
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
MIN_STOP_DISTANCE = 20  # cm
MAX_STOP_DISTANCE = 80  # cm
ALPHA = 0.2  # Amount to use current_speed in forward_speed

# Right and left points to sample in the depth image
RIGHT_POINT = (rc.camera.get_height() // 2, int(rc.camera.get_width() * 5.0 / 8.0))
LEFT_POINT = (rc.camera.get_height() // 2, int(rc.camera.get_width() * 3.0 / 8.0))

# Amount to increase stop distance (cm) per speed (cm/s) squared
STOP_DISTANCE_SCALE = 40.0 / 10000

# slow_distance / stop_distance
SLOW_DISTANCE_RATIO = 1.5

# >> Variables
forward_speed = 0  # cm/s
prev_distance = 0  # cm


########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global forward_speed
    global prev_distance

    # Have the car begin at a stop
    rc.drive.stop()

    # Initialize variables
    forward_speed = 0
    depth_image = rc.camera.get_depth_image()
    prev_distance = rc_utils.get_depth_image_center_distance(depth_image)

    # Print start message
    print(
        ">> Lab 3A - Depth Camera Safety Stop\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
        "   A button = print current speed and angle\n"
        "   B button = print the distance at the center of the depth image"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global forward_speed
    global prev_distance

    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    # Calculate the distance of the object directly in front of the car, sampling
    # multiple points and choosing the closest
    depth_image = rc.camera.get_depth_image()
    center_distance = rc_utils.get_depth_image_center_distance(depth_image)
    right_distance = rc_utils.get_pixel_average_distance(depth_image, RIGHT_POINT)
    left_distance = rc_utils.get_pixel_average_distance(depth_image, LEFT_POINT)

    # If any measurement in 0.0 (no data), make it the depth image max range
    if center_distance == 0.0:
        center_distance = rc.camera.get_max_range()
    if right_distance == 0.0:
        right_distance = rc.camera.get_max_range()
    if left_distance == 0.0:
        left_distance = rc.camera.get_max_range()

    distance = min(center_distance, right_distance, left_distance)

    # Update forward speed estimate
    cur_speed = (prev_distance - distance) / rc.get_delta_time()
    forward_speed += ALPHA * (cur_speed - forward_speed)
    prev_distance = distance

    # Calculate a stop distance based on the forward speed
    stop_distance = rc_utils.clamp(
        MIN_STOP_DISTANCE + forward_speed * abs(forward_speed) * STOP_DISTANCE_SCALE,
        MIN_STOP_DISTANCE,
        MAX_STOP_DISTANCE,
    )

    if not rc.controller.is_down(rc.controller.Button.RB):
        # If we are past slow_distance and driving forward, start to slow down
        slow_distance = stop_distance * SLOW_DISTANCE_RATIO
        if speed > 0 and stop_distance < distance < slow_distance:
            # Reduce speed proportional to how close we are to stop_distance
            speed = min(
                speed,
                rc_utils.remap_range(distance, stop_distance, slow_distance, 0, 0.5),
            )
            print("Safety slow: speed limited to {}".format(speed))

        # Safety stop if we are passed stop_distance
        if 0 < distance < stop_distance:
            # In this case, the car is already traveling in reverse, so we do not need
            # to add much additional reverse throttle
            if abs(stop_distance - MIN_STOP_DISTANCE) < 0.01:
                speed = -0.2

            # Reverse at a speed proportional to how far we are past stop_distance
            else:
                speed = rc_utils.remap_range(
                    distance, MIN_STOP_DISTANCE, stop_distance, -1, -0.2, True
                )
            print("Safety stop: reversing at {}".format(speed))

    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the depth image center distance when the B button is held down
    if rc.controller.is_down(rc.controller.Button.B):
        print("Center distance:", center_distance)

    # Print current forward_speed estimate when the X button is held down
    if rc.controller.is_down(rc.controller.Button.X):
        print("Forward speed estimate: {} cm/s".format(forward_speed))

    # Display the current depth image
    rc.display.show_depth_image(depth_image)

    # TODO (stretch goal): Prevent forward movement if the car is about to drive off a
    # ledge.  ONLY TEST THIS IN THE SIMULATION, DO NOT TEST THIS WITH A REAL CAR.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
