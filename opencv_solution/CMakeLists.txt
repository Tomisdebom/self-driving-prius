cmake_minimum_required(VERSION 2.8.3)
project(opencv_solution)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})


find_package(catkin REQUIRED COMPONENTS
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

add_executable(opencv_solution src/opencv_solution.cpp)
add_dependencies(opencv_solution ${catkin_EXPORTED_TARGETS})
target_link_libraries(opencv_solution ${OpenCV_LIBRARIES})
target_link_libraries(opencv_solution ${catkin_LIBRARIES})



