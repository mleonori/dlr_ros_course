cmake_minimum_required(VERSION 3.5.1)
project(dlr_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  message_generation
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  cv_bridge
  polled_camera
  diagnostic_updater
  camera_info_manager
  std_msgs
  visualization_msgs
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
    message_runtime
    gazebo_msgs
    roscpp
    rospy
    nodelet
    angles
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
    urdf
    tf
    tf2_ros
    dynamic_reconfigure
    rosgraph_msgs
    trajectory_msgs
    image_transport
    rosconsole
    camera_info_manager
    std_msgs
    visualization_msgs
)
add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})


# add_definitions(-fPIC) # what is this for?

## Plugins
add_library(gazebo_ros_planar_move src/gazebo_ros_planar_move.cpp)
target_link_libraries(gazebo_ros_planar_move ${catkin_LIBRARIES} ${Boost_LIBRARIES})
