cmake_minimum_required(VERSION 3.0.2)
project(homewhere)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

# Add this line to find the Gazebo package
find_package(gazebo REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Link against Gazebo and ROS libraries
add_library(control_plugin src/control_plugin.cpp)
target_link_libraries(control_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  LIBRARIES control_plugin
  CATKIN_DEPENDS roscpp geometry_msgs
)