"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

A simple program which can be used to manually test racecar_utils functionality.
"""

########################################################################################
# Imports
########################################################################################

import math
import sys

sys.path.insert(1, "../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

RED = ((170, 50, 50), (10, 255, 255))

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

    # Test numeric functions
    assert rc_utils.remap_range(5, 0, 10, 0, 50) == 25
    assert rc_utils.remap_range(5, 0, 20, 1000, 900) == 975
    assert rc_utils.remap_range(2, 0, 1, -10, 10) == 30
    assert rc_utils.remap_range(2, 0, 1, -10, 10, True) == 10

    assert rc_utils.clamp(3, 0, 10) == 3
    assert rc_utils.clamp(-2, 0, 10) == 0
    assert rc_utils.clamp(11, 0, 10) == 10

    # Print start message
    print(
        ">> Test Utils: A testing program for the racecar_utils library.\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
        "   A button = Take a color image and crop it to the top left\n"
        "   B button = Take a color image and identify the largest red contour\n"
        "   X button = Take a depth image and print several statistics\n"
        "   Y button = Take a lidar scan and print several statistics\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Display the color image cropped to the top left
    if rc.controller.was_pressed(rc.controller.Button.A):
        image = rc.camera.get_color_image()
        cropped = rc_utils.crop(
            image, (0, 0), (rc.camera.get_height() // 2, rc.camera.get_width() // 2)
        )
        rc.display.show_color_image(cropped)

    # Find and display the largest red contour in the color image
    if rc.controller.was_pressed(rc.controller.Button.B):
        image = rc.camera.get_color_image()
        contours = rc_utils.find_contours(image, RED[0], RED[1])
        largest_contour = rc_utils.get_largest_contour(contours)

        if largest_contour is not None:
            center = rc_utils.get_contour_center(largest_contour)
            area = rc_utils.get_contour_area(largest_contour)
            print("Largest red contour: center={}, area={:.2f}".format(center, area))
            rc_utils.draw_contour(image, largest_contour, rc_utils.ColorBGR.green.value)
            rc_utils.draw_circle(image, center, rc_utils.ColorBGR.yellow.value)
            rc.display.show_color_image(image)
        else:
            print("No red contours found")

    # Print depth image statistics and show the cropped upper half
    if rc.controller.was_pressed(rc.controller.Button.X):
        depth_image = rc.camera.get_depth_image()
        left_distance = rc_utils.get_pixel_average_distance(
            depth_image, (rc.camera.get_height() // 2, rc.camera.get_width() // 4),
        )
        center_distance = rc_utils.get_depth_image_center_distance(depth_image)
        center_distance_raw = rc_utils.get_depth_image_center_distance(depth_image, 1)
        right_distance = rc_utils.get_pixel_average_distance(
            depth_image, (rc.camera.get_height() // 2, 3 * rc.camera.get_width() // 4),
        )
        print("Depth image left distance: {:.2f} cm".format(left_distance))
        print("Depth image center distance: {:.2f} cm".format(center_distance))
        print("Depth image raw center distance: {:.2f} cm".format(center_distance_raw))
        print("Depth image right distance: {:.2f} cm".format(right_distance))

        cropped = rc_utils.crop(
            depth_image,
            (0, 0),
            (rc.camera.get_height() * 2 // 3, rc.camera.get_width()),
        )
        closest_point = rc_utils.get_closest_pixel(cropped)
        closest_distance = cropped[closest_point[0]][closest_point[1]]
        print(
            "Depth image closest point (upper half): (row={}, col={}), distance={:.2f} cm".format(
                closest_point[0], closest_point[1], closest_distance
            )
        )
        rc.display.show_depth_image(cropped, points=[closest_point])

    # Print lidar statistics and show visualization with closest point highlighted
    if rc.controller.was_pressed(rc.controller.Button.Y):
        lidar = rc.lidar.get_samples()
        front_distance = rc_utils.get_lidar_average_distance(lidar, 0)
        right_distance = rc_utils.get_lidar_average_distance(lidar, 90)
        back_distance = rc_utils.get_lidar_average_distance(lidar, 180)
        left_distance = rc_utils.get_lidar_average_distance(lidar, 270)
        print("Front LIDAR distance: {:.2f} cm".format(front_distance))
        print("Right LIDAR distance: {:.2f} cm".format(right_distance))
        print("Back LIDAR distance: {:.2f} cm".format(back_distance))
        print("Left LIDAR distance: {:.2f} cm".format(left_distance))

        closest_sample = rc_utils.get_lidar_closest_point(lidar)
        print(
            "Closest LIDAR point: {:.2f} degrees, {:.2f} cm".format(
                closest_sample[0], closest_sample[1]
            )
        )
        rc.display.show_lidar(lidar, highlighted_samples=[closest_sample])

    # Print lidar distance in the direction the right joystick is pointed
    rjoy_x, rjoy_y = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)
    if abs(rjoy_x) > 0 or abs(rjoy_y) > 0:
        lidar = rc.lidar.get_samples()
        angle = (math.atan2(rjoy_x, rjoy_y) * 180 / math.pi) % 360
        print(
            "LIDAR distance at angle {:.2f} = {:.2f} cm".format(
                angle, rc_utils.get_lidar_average_distance(lidar, angle)
            )
        )

    # Default drive-style controls
    left_trigger = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    right_trigger = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    left_joystick = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    rc.drive.set_speed_angle(right_trigger - left_trigger, left_joystick[0])


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
