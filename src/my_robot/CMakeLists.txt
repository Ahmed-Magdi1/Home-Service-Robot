cmake_minimum_required(VERSION 3.0.2)
project(my_robot)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  gazebo_ros
  sensor_msgs
  message_generation
  urdf
  xacro
  nav_msgs
  visualization_msgs
  move_base_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(pick_objects src/pick_objects.cpp)
target_link_libraries(pick_objects ${catkin_LIBRARIES})

add_executable(object_marker src/object_marker.cpp)
target_link_libraries(object_marker ${catkin_LIBRARIES})

add_executable(add_markers_alone src/add_markers_alone.cpp)
target_link_libraries(add_markers_alone ${catkin_LIBRARIES})

add_executable(pick_objects_alone src/pick_objects_alone.cpp)
target_link_libraries(pick_objects_alone ${catkin_LIBRARIES})
