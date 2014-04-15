cmake_minimum_required(VERSION 2.8.3)
project(joint_trajectory_action_tools)

find_package(catkin REQUIRED COMPONENTS 
  roslib 
  rospy
  joint_trajectory_action
  trajectory_msgs
  pr2_controllers_msgs)

catkin_package()

catkin_python_setup()
