"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# A queue of driving steps to execute
# Each entry is a list containing (time remaining, speed, angle)
queue = []

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()

    # Begin with an empty queue
    queue.clear()

    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
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

    # When the A button is pressed, add instructions to drive in a circle
    if rc.controller.was_pressed(rc.controller.Button.A):
        drive_circle()

    # When the B button is pressed, add instructions to drive in a square
    if rc.controller.was_pressed(rc.controller.Button.B):
        drive_square()

    # When the X button is pressed, add instructions to drive in a figure eight
    if rc.controller.was_pressed(rc.controller.Button.X):
        drive_figure_eight()

    # TODO: Drive in a shape of your choice when the Y button is pressed

    # Calculate speed from triggers
    right_trigger = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    left_trigger = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = right_trigger - left_trigger

    # Calculate angle from left joystick
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    # If the triggers or joystick were pressed, clear the queue to cancel the current
    # shape and allow for manual driving
    if right_trigger > 0 or right_trigger > 0 or angle > 0:
        queue.clear()

    # If the queue is not empty, follow the current drive instruction
    if len(queue) > 0:
        speed = queue[0][1]
        angle = queue[0][2]
        queue[0][0] -= rc.get_delta_time()
        if queue[0][0] <= 0:
            queue.pop(0)

    rc.drive.set_speed_angle(speed, angle)

    # TODO (main challenge): Drive in a shape of your choice when the Y button
    # is pressed


def drive_circle():
    """
    Add steps to drive in a circle to the instruction queue.
    """
    global queue

    # Tune this constant until the car completes a full circle
    CIRCLE_TIME = 6

    queue.clear()

    # Turn right at full speed
    queue.append([CIRCLE_TIME, 1, 1])


def drive_square():
    """
    Add steps to drive in a square to the instruction queue.
    """
    global queue

    # Tune these constants until the car completes a clean square
    STRAIGHT_TIME = 1.5
    TURN_TIME = 1.2

    queue.clear()

    # Repeat 4 copies of: drive straight, turn right
    for i in range(0, 4):
        queue.append([STRAIGHT_TIME, 1, 0])
        queue.append([TURN_TIME, 1, 1])


def drive_figure_eight():
    """
    Add steps to drive in a figure eight to the instruction queue.
    """
    global queue

    # Tune these constants until the car completes a clean figure eight
    STRAIGHT_TIME_1 = 3  # The first straight time is longer to give time to accelerate
    STRAIGHT_TIME_2 = 2
    TURN_TIME = 3.5

    queue.clear()

    # Take the following steps: straight, right, straight, left
    queue.append([STRAIGHT_TIME_1, 1, 0])
    queue.append([TURN_TIME, 1, 1])
    queue.append([STRAIGHT_TIME_2, 1, 0])
    queue.append([TURN_TIME, 1, -1])


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
