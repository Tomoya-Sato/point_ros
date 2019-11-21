#ifndef RING_FILTER_H
#define RING_FILTER_H

#include <iostream>
#include <vector>
#include <chrono>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <velodyne_pointcloud/point_types.h>

class RingFilter
{
  public:
    RingFilter();
    ~RingFilter();
    void run();

  private:
    // ROS
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher filtered_pub_;
    ros::Subscriber points_sub_;

    // Params
    int ray_number_;
    float max_leafsize_, min_leafsize_;

    // Callback functions
    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Functions
};

#endif
