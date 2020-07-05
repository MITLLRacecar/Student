"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Phase 1 Challenge - Cone Slaloming
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import enum

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()


class Mode(enum.IntEnum):
    red_align = 0  # Approaching a red cone to pass
    blue_align = 1  # Approaching a blue cone to pass
    red_pass = 2  # Passing a red cone (currently out of sight to our left)
    blue_pass = 3  # Passing a blue cone (currently out of sight to our right)
    red_find = 4  # Finding a red cone with which to align
    blue_find = 5  # Finding a blue cone with which to align
    red_reverse = 6  # Aligning with a red cone, but we are too close so must back up
    blue_reverse = 7  # Aligning with a blue cone, but we are too close so must back up
    no_cones = 8  # No cones in sight, inch forward until we find one


# >> Constants
# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((100, 150, 150), (120, 255, 255))  # The HSV range for the color blue
RED = ((170, 50, 50), (10, 255, 255))  # The HSV range for the color blue

# Speeds
MAX_ALIGN_SPEED = 0.8
MIN_ALIGN_SPEED = 0.4
PASS_SPEED = 0.5
FIND_SPEED = 0.2
REVERSE_SPEED = -0.2
NO_CONES_SPEED = 0.4

# Times
REVERSE_BRAKE_TIME = 0.25
SHORT_PASS_TIME = 1.0
LONG_PASS_TIME = 1.2

# Cone finding parameters
MIN_CONTOUR_AREA = 100
MAX_DISTANCE = 250
REVERSE_DISTANCE = 50
STOP_REVERSE_DISTANCE = 60

CLOSE_DISTANCE = 30
FAR_DISTANCE = 120

# >> Variables
cur_mode = Mode.no_cones
counter = 0
red_center = None
red_distance = 0
prev_red_distance = 0
blue_center = None
blue_distance = 0
prev_blue_distance = 0

########################################################################################
# Functions
########################################################################################


def find_cones():
    """
    Find the closest red and blue cones and update corresponding global variables.
    """
    global red_center
    global red_distance
    global prev_red_distance
    global blue_center
    global blue_distance
    global prev_blue_distance

    prev_red_distance = red_distance
    prev_blue_distance = blue_distance

    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()

    if color_image is None or depth_image is None:
        red_center = None
        red_distance = 0
        blue_center = None
        blue_distance = 0
        print("No image found")
        return

    # Search for the red cone
    contours = rc_utils.find_contours(color_image, RED[0], RED[1])
    contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

    if contour is not None:
        red_center = rc_utils.get_contour_center(contour)
        red_distance = rc_utils.get_pixel_average_distance(depth_image, red_center)

        # Only use count it if the cone is less than MAX_DISTANCE away
        if red_distance <= MAX_DISTANCE:
            rc_utils.draw_contour(color_image, contour, rc_utils.ColorBGR.green.value)
            rc_utils.draw_circle(color_image, red_center, rc_utils.ColorBGR.green.value)
        else:
            red_center = None
            red_distance = 0
    else:
        red_center = None
        red_distance = 0

    # Search for the blue cone
    contours = rc_utils.find_contours(color_image, BLUE[0], BLUE[1])
    contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

    if contour is not None:
        blue_center = rc_utils.get_contour_center(contour)
        blue_distance = rc_utils.get_pixel_average_distance(depth_image, blue_center)

        # Only use count it if the cone is less than MAX_DISTANCE away
        if blue_distance <= MAX_DISTANCE:
            rc_utils.draw_contour(color_image, contour, rc_utils.ColorBGR.yellow.value)
            rc_utils.draw_circle(
                color_image, blue_center, rc_utils.ColorBGR.yellow.value
            )
        else:
            blue_center = None
            blue_distance = 0
    else:
        blue_center = None
        blue_distance = 0

    rc.display.show_color_image(color_image)


def start():
    """
    This function is run once every time the start button is pressed
    """
    global cur_mode
    global counter

    # Have the car begin at a stop, in no_cones mode
    rc.drive.stop()
    cur_mode = Mode.no_cones
    counter = 0

    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global cur_mode
    global counter

    find_cones()

    angle: float
    speed: float

    if cur_mode == Mode.red_align:
        if (
            red_center is None
            or red_distance == 0
            or red_distance - prev_red_distance > CLOSE_DISTANCE
        ):
            if 0 < prev_red_distance < FAR_DISTANCE:
                counter = max(1, counter)
                cur_mode = Mode.red_pass
            else:
                cur_mode = Mode.no_cones
        elif (
            red_distance < REVERSE_DISTANCE
            and red_center[1] > rc.camera.get_width() // 4
        ):
            counter = REVERSE_BRAKE_TIME
            cur_mode = Mode.red_reverse
        else:
            goal_point = rc_utils.remap_range(
                red_distance,
                CLOSE_DISTANCE,
                FAR_DISTANCE,
                0,
                rc.camera.get_width() // 4,
                True,
            )

            angle = rc_utils.remap_range(
                red_center[1], goal_point, rc.camera.get_width() // 2, 0, 1
            )
            angle = rc_utils.clamp(angle, -1, 1)

            speed = rc_utils.remap_range(
                red_distance,
                CLOSE_DISTANCE,
                FAR_DISTANCE,
                MIN_ALIGN_SPEED,
                MAX_ALIGN_SPEED,
                True,
            )

    elif cur_mode == Mode.blue_align:
        if (
            blue_center is None
            or blue_distance == 0
            or blue_distance - prev_blue_distance > CLOSE_DISTANCE
        ):
            if 0 < prev_blue_distance < FAR_DISTANCE:
                counter = max(1, counter)
                cur_mode = Mode.blue_pass
            else:
                cur_mode = Mode.no_cones
        elif (
            blue_distance < REVERSE_DISTANCE
            and blue_center[1] < rc.camera.get_width() * 3 // 4
        ):
            counter = REVERSE_BRAKE_TIME
            cur_mode = Mode.blue_reverse
        else:
            goal_point = rc_utils.remap_range(
                blue_distance,
                CLOSE_DISTANCE,
                FAR_DISTANCE,
                rc.camera.get_width(),
                rc.camera.get_width() * 3 // 4,
                True,
            )

            angle = rc_utils.remap_range(
                blue_center[1], goal_point, rc.camera.get_width() // 2, 0, -1
            )
            angle = rc_utils.clamp(angle, -1, 1)

            speed = rc_utils.remap_range(
                blue_distance,
                CLOSE_DISTANCE,
                FAR_DISTANCE,
                MIN_ALIGN_SPEED,
                MAX_ALIGN_SPEED,
                True,
            )

    if cur_mode == Mode.red_pass:
        angle = rc_utils.remap_range(counter, 1, 0, 0, -0.5)
        speed = PASS_SPEED

        counter -= rc.get_delta_time()
        if counter <= 0:
            cur_mode = Mode.blue_align if blue_distance > 0 else Mode.blue_find

    elif cur_mode == Mode.blue_pass:
        angle = rc_utils.remap_range(counter, 1, 0, 0, 0.5)
        speed = PASS_SPEED

        counter -= rc.get_delta_time()
        if counter <= 0:
            cur_mode = Mode.red_align if red_distance > 0 else Mode.red_find

    elif cur_mode == Mode.red_find:
        angle = 1
        speed = FIND_SPEED
        if red_distance > 0:
            cur_mode = Mode.red_align

    elif cur_mode == Mode.blue_find:
        angle = -1
        speed = FIND_SPEED
        if blue_distance > 0:
            cur_mode = Mode.blue_align

    elif cur_mode == Mode.red_reverse:
        if counter >= 0:
            counter -= rc.get_delta_time()
            speed = -1
            angle = 1
        else:
            angle = -1
            speed = REVERSE_SPEED
            if (
                red_distance > STOP_REVERSE_DISTANCE
                or red_center[1] < rc.camera.get_width() // 10
            ):
                counter = LONG_PASS_TIME
                cur_mode = Mode.red_align

    elif cur_mode == Mode.blue_reverse:
        if counter >= 0:
            counter -= rc.get_delta_time()
            speed = -1
            angle = 1
        else:
            angle = 1
            speed = REVERSE_SPEED
            if (
                blue_distance > STOP_REVERSE_DISTANCE
                or blue_center[1] > rc.camera.get_width() * 9 / 10
            ):
                counter = LONG_PASS_TIME
                cur_mode = Mode.blue_align

    elif cur_mode == Mode.no_cones:
        angle = 0
        speed = NO_CONES_SPEED

        if red_distance > 0 and blue_distance == 0:
            cur_mode = Mode.red_align
        elif blue_distance > 0 and red_distance == 0:
            cur_mode = Mode.blue_align
        elif blue_distance > 0 and red_distance > 0:
            cur_mode = (
                Mode.red_align if red_distance < blue_distance else Mode.blue_align
            )

    rc.drive.set_speed_angle(speed, angle)

    print(
        f"Mode: {cur_mode.name}, red_distance: {red_distance:.2f} cm, blue_distance: {blue_distance:.2f} cm, speed: {speed:.2f}, angle: {angle:2f}"
    )


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
