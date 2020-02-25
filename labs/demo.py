"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Demo RACECAR program
"""

################################################################################
# Imports
################################################################################

import sys
sys.path.insert(0, '../library')
from racecar_core import *
rospy.init_node('racecar')

################################################################################
# Global variables
################################################################################

rc = Racecar()

# Declare any global variables here
counter = 0
isDriving = False

################################################################################
# Functions
################################################################################

def start():
    """
    This function is run once every time the start button is pressed
    """
    # If we use a global variable in our function, we must list it at
    # the beginning of our function like this
    global counter
    global isDriving

    # The start function is a great place to give initial values to globals
    counter = 0
    isDriving = False

    # This tells the car to begin at a standstill
    rc.drive.stop()

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global counter
    global isDriving

    # This prints a message every time the A button is pressed on the controller
    if rc.controller.was_pressed(rc.controller.Button.A):
      print("The A button was pressed")

    # Reset the counter and start driving every time the B button is pressed on
    # the controller
    if rc.controller.was_pressed(rc.controller.Button.B):
      counter = 0
      isDriving = True

    if isDriving:
      # rc.get_delta_time() gives the time in seconds since the last time
      # the update function was called
      counter += rc.get_delta_time()

      if counter < 1:
        # Drive forward at full speed for one second
        rc.drive.set_speed_angle(1, 0)
      elif counter < 2:
        # Turn left at full speed for the next second
        rc.drive.set_speed_angle(1, 1)
      else:
        # Otherwise, stop the car
        rc.drive.stop()
        isDriving = False

    # Take and display a photo every time the X button is pressed on the
    # controller
    if rc.controller.was_pressed(rc.controller.Button.X):
        # Capture an image from the RACECAR camera
        image = rc.camera.get_image()

        # If an image was captured, show it on the RACECAR monitor
        if image is not None:
            rc.display.show_image(image)
        else:
            print("Error: No image was captured")


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
