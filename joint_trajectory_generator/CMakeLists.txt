# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(joint_trajectory_generator)

find_package(catkin REQUIRED COMPONENTS actionlib angles joint_trajectory_action roscpp urdf)

add_library(joint_trajectory_generation src/trajectory_generation.cpp)

add_executable(joint_trajectory_generator src/joint_trajectory_generator.cpp)
target_link_libraries(joint_trajectory_generator joint_trajectory_generation)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(orocos_kdl REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(joint_trajectory_generator ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES})

catkin_package(
    DEPENDS orocos_kdl 
    CATKIN-DEPENDS actionlib angles joint_trajectory_action roscpp urdf
    INCLUDE_DIRS include
    LIBRARIES joint_trajectory_generation
)