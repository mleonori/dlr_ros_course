cmake_minimum_required(VERSION 3.5.1)
project(dlr_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_msgs
  roscpp
  rospy
)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS
    gazebo_msgs
    roscpp
    rospy
)

if (UNIX)
  ## Plugins
  add_library(dlr_gazebo_ros_planar_move src/gazebo_ros_planar_move.cpp)
  target_link_libraries(dlr_gazebo_ros_planar_move ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif (UNIX)