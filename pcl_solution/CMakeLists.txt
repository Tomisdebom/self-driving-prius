cmake_minimum_required(VERSION 2.8.3)
project(pcl_solution)

find_package(OpenCV)
find_package(PCL 1.2 REQUIRED)

link_directories(${PCL_INCLUDE_DIRS})



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs 
  sensor_msgs
  pcl_ros
  pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
catkin_package(
)

#catkin_package(PORTED_TARGETS})
add_executable(pcl_solution_node src/pcl_solution_node.cpp)
add_dependencies(pcl_solution_node ${catkin_EXPORTED_TARGETS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(pcl_solution_node ${OpenCV_LIBRARIES})
target_link_libraries(pcl_solution_node ${catkin_LIBRARIES})
target_link_libraries(pcl_solution_node ${PCL_LIBRARIES})

