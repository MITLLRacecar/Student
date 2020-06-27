"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 4B - IMU: Driving in shapes
"""

################################################################################
# Imports
################################################################################

import sys

sys.path.insert(0, "../../library")
from racecar_core import *
import racecar_utils as rc_utils

import cv2 as cv
import numpy as np
import math


################################################################################
# Global variables
################################################################################

rc = racecar_core.create_racecar()


# A queue of driving steps to execute
# Each entry is a triple of the form (radians/distance remaining, speed, angle)
queue = []

# # Keep track of velocity to better estimate distance
old_linear_velocity = 0
current_linear_velocity = 0

# Distance between wheels in meters
WHEEL_DISTANCE = 0.2286

################################################################################
# Functions
################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global queue
    global old_linear_velocity
    global current_linear_velocity

    # Begin at a full stop
    rc.drive.stop()

    # Begin with an empty queue
    queue.clear()

    old_linear_velocity = 0
    current_linear_velocity = 0

    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Lab 4B - IMU: Driving in Shapes \n"
        "\n"
        "Controlls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
        "   A button = drive in a circle\n"
        "   B button = drive in a square\n"
        "   X button = drive in a figure eight\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global queue
    global old_linear_velocity
    global current_linear_velocity

    # Calculate speed from triggers
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = forwardSpeed - backSpeed

    # Calculate angle from left joystick
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    # TODO (main challenge): Revisit the driving in shapes challenge.
    # Using your IMU data, create a more robust way to drive in shapes.

    # When the A button is pressed, add instructions to drive in a circle
    if rc.controller.was_pressed(rc.controller.Button.A):
        # TODO (main challenge): drive in a circle
        drive_circle()

    # When the B button is pressed, add instructions to drive in a square
    if rc.controller.was_pressed(rc.controller.Button.B):
        # TODO (main challenge): drive in a square
        drive_square()

    # When the X button is pressed, add instructions to drive in a figure eight
    if rc.controller.was_pressed(rc.controller.Button.X):
        # TODO (main challenge): drive in a figure eight
        drive_figure_eight()

    # When the Y button is pressed, add instructions to drive in a shape of your choice
    if rc.controller.was_pressed(rc.controller.Button.X):
        # TODO (main challenge): drive in the shape of your choice
        pass

    # If the triggers or joystick were pressed, clear the queue to cancel the current
    # shape and allow for manual driving
    if forwardSpeed > 0 or backSpeed > 0 or angle > 0:
        queue.clear()

    # Retrieve imu data
    ang_vel = rc.physics.get_angular_velocity()
    lin_acc = rc.physics.get_linear_acceleration()

    # If the queue is not empty, follow the current drive instruction
    if len(queue) > 0:
        speed = queue[0][1]
        angle = queue[0][2]
        # If we are turning, update the radians left to turn
        if abs(angle) == 1 and speed == 1:
            radians_left = queue[0][0] - (abs(ang_vel[1]) * rc.get_delta_time())
            queue[0] = (radians_left, speed, angle)
            current_linear_velocity = (
                abs(ang_vel[1]) * WHEEL_DISTANCE / math.sin(math.pi / 4)
            )
        # If we are driving straight, update distance left to travel
        if abs(angle) == 0 and speed == 1:
            # Displacement can be calculated using the following equations
            # s = v_avg * Δt, s.t. v_avg = (v_i + v_f) / 2 and v_f = v_i + a * Δt, or
            # s = v_i * Δt + (1/2) * a * (Δt)**2
            current_linear_velocity = (
                old_linear_velocity + lin_acc[2] * rc.get_delta_time()
            )
            avg_linear_velocity = (old_linear_velocity + current_linear_velocity) / 2
            dist_traveled = avg_linear_velocity * rc.get_delta_time()
            dist_left = queue[0][0] - dist_traveled
            queue[0] = (dist_left, speed, angle)
        if queue[0][0] <= 0:
            print("Pop")
            queue.pop(0)

    # Update the old veloxcity value
    old_linear_velocity = current_linear_velocity

    # Set the final speed and angle
    rc.drive.set_speed_angle(speed, angle)


def drive_circle():
    """
    Add steps to drive in a circle to the instruction queue
    """
    global queue

    # The number of radians to turn a full circle
    RADS_LEFT = math.pi * 2

    queue.clear()

    # Stop driving
    queue.append((0, 0, 0))
    # Turn left at full speed
    queue.append((RADS_LEFT, 1, 1))


def drive_square():
    """
    Add steps to drive in a square to the instruction queue
    """
    global queue

    # Tune these constants until the car completes a clean square
    DIST_LEFT = 0.2
    RADS_LEFT = math.pi / 2

    queue.clear()

    # Stop driving
    queue.append((0, 0, 0))

    # Repeat 4 copies of: drive straight, turn left
    for i in range(1, 4):
        queue.append((RADS_LEFT, 1, 1))
        queue.append((DIST_LEFT, 1, 0))


def drive_figure_eight():
    """
    Add steps to drive in a figure eight to the instruction queue
    """
    global queue

    # Tune these constants until the car completes a clean figure eight
    DIST_LEFT = 0.2
    RADS_LEFT = math.pi * 1.5

    queue.clear()

    # Stop driving
    queue.append((0, 0, 0))

    # Turn left, drive straight, turn right, drive straight
    queue.append((RADS_LEFT, 1, 1))
    queue.append((DIST_LEFT, 1, 0))
    queue.append((RADS_LEFT, 1, -1))
    queue.append((DIST_LEFT, 1, 0))


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
