"""
Copyright Harvey Mudd College
Fall 2019

Lab <n> - <Lab Title>
"""

################################################################################
# Imports
################################################################################

import sys
sys.path.insert(0, '../../library')
from racecar_core import *
rospy.init_node('racecar')


################################################################################
# Global variables
################################################################################

rc = Racecar()


################################################################################
# Functions
################################################################################

def start():
    """
    This function is run once every time the start button is pressed
    """
    pass

def update():
    """
    After start() is run, this function is run every frame until the back button is
    pressed
    """
    pass


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.run(start, update)
