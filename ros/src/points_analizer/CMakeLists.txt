cmake_minimum_required(VERSION 2.8.3)
project(points_analizer)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

#Enable support for c++11
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else ()
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(points_analizer
  nodes/points_analizer/points_analizer.cpp
  nodes/points_analizer/points_analizer_node.cpp
)
target_link_libraries(points_analizer ${catkin_LIBRARIES})
add_dependencies(points_analizer ${catkin_EXPORTED_TARGETS})
