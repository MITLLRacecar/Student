#usr/bin/python

# node for arbitrating who has drive control

import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

# denotes which messages can pass through the mux
# '' -> nothing passes through
# 'gamepad' -> gamepad commands pass through
# 'autonomy' -> drive commands pass through
# other -> ignore
mux_mode = ''

# gamepad callback
def joy_callback(msg):
    global mux_mode

    # if LB is pressed, enable teleop
    if msg.buttons[4] == 1:
        mux_mode = 'gamepad'
    # if RB is pressed, enable autonomy
    elif msg.buttons[5] == 1:
        mux_mode = 'autonomy'
    # otherwise default, publish stop
    else:
        mux_mode = ''
        mux_out_pub.publish(AckermannDriveStamped())


# callback for gamepad_drive topic
def gamepad_drive_callback(msg):
    global mux_mode, mux_out_pub
    if mux_mode == 'gamepad':
        mux_out_pub.publish(msg)

# callback for drive topic
def drive_callback(msg):
    global mux_mode, mux_out_pub
    if mux_mode == 'autonomy':
        mux_out_pub.publish(msg)

# init ROS
rospy.init_node('mux')
mux_out_pub = rospy.Publisher('/mux_out', AckermannDriveStamped, queue_size=1)
rospy.Subscriber('/gamepad_drive', AckermannDriveStamped, gamepad_drive_callback)
rospy.Subscriber('/drive', AckermannDriveStamped, drive_callback)
rospy.Subscriber('/joy', Joy, joy_callback)

# wait before shutdown
rospy.spin()
