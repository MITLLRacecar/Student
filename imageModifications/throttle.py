#!/usr/bin/python

# node for ensuring commanded speed does not exceed throttle

import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

# get param file values
CAR_THROTTLE_FORWARD = rospy.get_param('car_throttle_forward')
CAR_THROTTLE_BACKWARD = rospy.get_param('car_throttle_backward')
CAR_THROTTLE_TURN = rospy.get_param('car_throttle_turn')

# callback that throttles max speed and angle
def drive_callback(msg):
    global motor_pub
    if msg.drive.speed > CAR_THROTTLE_FORWARD:
        msg.drive.speed = CAR_THROTTLE_FORWARD

    if msg.drive.speed < -CAR_THROTTLE_BACKWARD:
        msg.drive.speed = -CAR_THROTTLE_BACKWARD

    if msg.drive.steering_angle > CAR_THROTTLE_TURN:
        msg.drive.steering_angle = CAR_THROTTLE_TURN

    if msg.drive.steering_angle < -CAR_THROTTLE_TURN:
        msg.drive.steering_angle = -CAR_THROTTLE_TURN 

    motor_pub.publish(msg)

# init ROS
rospy.init_node('throttle')
motor_pub = rospy.Publisher('/motor', AckermannDriveStamped, queue_size=1)
rospy.Subscriber('/mux_out', AckermannDriveStamped, drive_callback)

# wait before shutdown
rospy.spin()
