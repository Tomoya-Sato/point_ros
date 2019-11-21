#ifndef VOXEL_GRID_FILTER_H
#define VOXEL_GRID_FILTER_H

#include <iostream>
#include <chrono>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

class VoxelGridFilter
{
  public:
    VoxelGridFilter();
    ~VoxelGridFilter();
    void run();

  private:
    // ROS
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher filtered_pub_;
    ros::Subscriber points_sub_;

    ros::Publisher app_pub_;

    // Params
    double leafsize_;
    int method_;

    // Callback funcitons
    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};

#endif
