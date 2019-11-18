#ifndef DISTANCE_VOXEL_FILTER_H
#define DISTNACE_VOXEL_FILTER_H

#include <iostream>
#include <vector>
#include <chrono>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <velodyne_pointcloud/point_types.h>

class DistanceVoxelFilter
{
  public:
    DistanceVoxelFilter();
    ~DistanceVoxelFilter();
    void run();

  private:
    // ROS
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher filtered_pub_;
    ros::Subscriber points_sub_;

    // Params
    double max_range_;
    double near_leafsize_, dist_leafsize_;

    // Callback functinos
    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Classes
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
};

#endif
