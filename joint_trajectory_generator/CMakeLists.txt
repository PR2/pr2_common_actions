cmake_minimum_required(VERSION 2.8.3)
project(joint_trajectory_generator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  angles
  kdl_parser
  joint_trajectory_action
  roscpp
  urdf
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES joint_trajectory_generation
#  CATKIN_DEPENDS actionlib actionlib_msgs pr2_common_action_msgs pr2_controllers_msgs rospy trajectory_msgs
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(joint_trajectory_generation src/trajectory_generation.cpp)
target_link_libraries(joint_trajectory_generation ${catkin_LIBRARIES})

add_executable(joint_trajectory_generator src/joint_trajectory_generator.cpp)
target_link_libraries(joint_trajectory_generator joint_trajectory_generation ${catkin_LIBRARIES})

install(TARGETS joint_trajectory_generator joint_trajectory_generation
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

