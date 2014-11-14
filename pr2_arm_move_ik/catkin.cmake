cmake_minimum_required(VERSION 2.8.3)
project(pr2_arm_move_ik)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  pr2_arm_kinematics
  pr2_common_action_msgs
  pr2_controllers_msgs
  roscpp
  tf
  urdf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm_ik src/arm_ik.cpp)
add_executable(pr2_arm_ik_test src/test_clients/pr2_arm_ik_test.cpp)

target_link_libraries(arm_ik
  ${catkin_LIBRARIES}
)

target_link_libraries(pr2_arm_ik_test
  ${catkin_LIBRARIES}
)

install(TARGETS arm_ik pr2_arm_ik_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
