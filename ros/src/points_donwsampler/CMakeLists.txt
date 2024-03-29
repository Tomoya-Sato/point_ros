cmake_minimum_required(VERSION 2.8.3)
project(points_downsampler)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

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

catkin_package()


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ring_filter
  nodes/ring_filter/ring_filter.cpp
  nodes/ring_filter/ring_filter_node.cpp
  nodes/ring_filter/voxel_grid_approxi.cpp
)
target_link_libraries(ring_filter ${catkin_LIBRARIES})
add_dependencies(ring_filter ${catkin_EXPORTED_TARGETS})

add_executable(ring_random_filter
  nodes/ring_random_filter/ring_random_filter.cpp
  nodes/ring_random_filter/ring_random_filter_node.cpp
)
target_link_libraries(ring_random_filter ${catkin_LIBRARIES})
add_dependencies(ring_random_filter ${catkin_EXPORTED_TARGETS})

add_executable(distance_voxel_filter
  nodes/distance_voxel_filter/distance_voxel_filter.cpp
  nodes/distance_voxel_filter/distance_voxel_filter_node.cpp
  nodes/distance_voxel_filter/voxel_grid_approxi.cpp
)
target_link_libraries(distance_voxel_filter ${catkin_LIBRARIES})
add_dependencies(distance_voxel_filter ${catkin_EXPORTED_TARGETS})


add_executable(voxel_grid_filter
  nodes/voxel_grid_filter/voxel_grid_filter.cpp
  nodes/voxel_grid_filter/voxel_grid_filter_node.cpp
  nodes/voxel_grid_filter/voxel_grid_approxi.cpp
)
target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})
add_dependencies(voxel_grid_filter ${catkin_EXPORTED_TARGETS})

add_executable(random_filter
  nodes/random_filter/random_filter.cpp
  nodes/random_filter/random_filter_node.cpp
)
target_link_libraries(random_filter ${catkin_LIBRARIES})
add_dependencies(random_filter ${catkin_EXPORTED_TARGETS})
