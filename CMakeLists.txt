cmake_minimum_required(VERSION 3.0.2)
project(movegroup_ar)

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  geometry_msgs
  moveit_ros_planning_interface
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ar_track_alvar geometry_msgs moveit_ros_planning_interface roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_goal_ar src/pose_goal_ar.cpp)
target_link_libraries(pose_goal_ar ${catkin_LIBRARIES})
