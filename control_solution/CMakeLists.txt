cmake_minimum_required(VERSION 2.8.3)
project(control_solution)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})


find_package(catkin REQUIRED COMPONENTS
  prius_msgs
  roscpp
  rospy
  std_msgs
  image_transport 
  sensor_msgs
  cv_bridge
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(control_solution_node src/control_solution_node.cpp)
add_dependencies(control_solution_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_solution_node ${OpenCV_LIBRARIES})
target_link_libraries(control_solution_node ${catkin_LIBRARIES})



