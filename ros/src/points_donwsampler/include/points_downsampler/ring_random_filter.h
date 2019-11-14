#ifndef RING_RANDOM_FILTER_H
#define RING_RANDOM_FILTER_H

#include <iostream>
#include <vector>
#include <chrono>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <velodyne_pointcloud/point_types.h>

class RingRandomFilter
{
  public:
    RingRandomFilter();
    ~RingRandomFilter();
    void run;

  private:
    // ROS
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher filtered_pub_;
    ros::Subscriber points_sub_;

    // Params
    int ray_number_;
    
    // Callback functions
    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Functions
    bool orderPoints(const pcl::PointXYZI& p0, const pcl::PointXYZI& p1);
    bool inversePoints(const pcl::PointXYZI& p0, const pcl::PointXYZI& p1);
};

#endif
