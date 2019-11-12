#ifndef POINTS_ANALIZER_H
#define POINTS_ANALIZER_H

#include <iostream>
#include <fstream>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

class PointsAnalizer
{
  public:
    PointsAnalizer();
    ~PointsAnalizer();
    void run();

  private:
    // ROS
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber points_sub_;
    ros::Subscriber end_sub_;

    // Params
    std::string log_name_;

    // Variables
    std::ofstream ofs_;
    std::string points_name_;

    // Callback functions
    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void endCallback(const std_msgs::Bool msg);

    // Functions
};

#endif
