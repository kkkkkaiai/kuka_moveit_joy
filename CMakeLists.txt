cmake_minimum_required(VERSION 2.8.3)
project(kuka_moveit_joy)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  iiwa_description
  iiwa_moveit
  iiwa_msgs
  roscpp
  rosmsg
  rospy
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  moveit_msgs
  moveit_visual_tools
  control_msgs
  sensor_msgs
  actionlib
  tf
  joy
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    iiwa_description
    iiwa_moveit
    iiwa_msgs
    roscpp
    rosmsg
    rospy
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    moveit_msgs
    moveit_visual_tools
    control_msgs
    sensor_msgs
    actionlib
    joy
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(control ./src/control.cpp)

target_link_libraries(control
  ${catkin_LIBRARIES}
)
