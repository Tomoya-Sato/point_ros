cmake_minimum_required(VERSION 2.8.3)
project(points_downsampler)

find_package(PCL REQUIRED)

# Enable support for c++11
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  velodyne_pointcloud
)

catkin_package(DEPENDS PCL)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ring_filter
  nodes/ring_filter/ring_filter.cpp
  nodes/ring_filter/ring_filter_node.cpp
)
target_link_libraries(ring_filter ${catkin_LIBRARIES})
add_dependencies(ring_filter ${catkin_EXPORTED_TARGETS})
