#!/usr/bin/python

# node for turning gamepad inputs into drive commands

import rospy
from sensor_msgs.msg import Joy

core_callback = None

def set_core_callback(callback):
    print("set_core_callback called")
    global core_callback
    core_callback = callback

def controller_callback(msg):
    print("gamepad controller callback called")
    if core_callback is not None:
        core_callback(msg)

if __name__ == "__main__":
    # init ROS
    print("before gamepad")
    rospy.init_node('gamepad')
    print("after gamepad")
    rospy.Subscriber('/joy', Joy, controller_callback)

    # wait before shutdown
    rospy.spin()
