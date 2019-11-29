cmake_minimum_required(VERSION 2.8.3)
project(kitti2rosbag)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

# Enable support for c++11
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=11")
else ()
  set(CMAKE_CXX_STANDARD 11)
endif ()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(velodyne2bag nodes/velodyne2bag/velodyne2bag.cpp)
target_link_libraries(velodyne2bag ${catkin_LIBRARIES})
add_dependencies(velodyne2bag ${catkin_EXPORTED_TARGETS})
