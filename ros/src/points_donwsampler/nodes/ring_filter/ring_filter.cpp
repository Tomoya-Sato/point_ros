#include "points_downsampler/ring_filter.h"

RingFilter::RingFilter() : nh_(), private_nh_("~")
{
  private_nh_.param<int>("ray_number", ray_number_, 32);

  filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_points", 10);
  points_sub_ = nh_.subscribe("points_raw", 1, &RingFilter::pointsCallback, this);
}

RingFilter::~RingFilter()
{
}

void RingFilter::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> input;
  pcl::PointCloud<pcl::PointXYZI> output;
  pcl::PointCloud<pcl::PointXYZI> tmp;

  pcl::fromROSMsg(*msg, input);

  for (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator item = input.begin(); item != input.end(); item++)
  {
    if (item->ring == 0)
    {
      pcl::PointXYZI p;
      p.x = item->x;
      p.y = item->y;
      p.z = item->z;
      p.intensity = item->intensity;

      tmp.push_back(p);
    }
  }

  sensor_msgs::PointCloud2 filtered_msg;
  pcl::toROSMsg(output, filtered_msg);

  filtered_msg.header = msg->header;
  filtered_pub_.publish(filtered_msg);
}

void RingFilter::run()
{
  ros::spin();
}
