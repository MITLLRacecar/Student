cmake_minimum_required(VERSION 2.8.3)
project(racecar_mn)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package()
