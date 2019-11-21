#ifndef RANDOM_FILTER_H
#define RANDOM_FILTER_H

#include <iostream>
#include <chrono>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

class RandomFilter
{
  public:
    RandomFilter();
    ~RandomFilter();
    void run();

  private:
    // ROS
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher filtered_pub_;
    ros::Subscriber points_sub_;

    // Params
    int points_number_;

    // Callback functions
    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};

#endif
