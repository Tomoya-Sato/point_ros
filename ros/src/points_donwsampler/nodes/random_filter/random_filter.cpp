#include "points_downsampler/random_filter.h"

RandomFilter::RandomFilter() : nh_(), private_nh_("~")
{
  private_nh_.param<int>("points_number", points_number_, 1500);

  filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_points", 10);
  points_sub_ = nh_.subscribe("points_raw", 1, &RandomFilter::pointsCallback, this);
}

RandomFilter::~RandomFilter()
{
}

void RandomFilter::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI> input;
  pcl::PointCloud<pcl::PointXYZI> output;

  std::ofstream ofs;
  ofs.open("ring_random_time.csv", std::ios::app);

  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  pcl::fromROSMsg(*msg, input);

  int stride = 32;

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = input.begin(); item <= input.end(); item+=stride)
  {
    const pcl::PointXYZI& p = *item;

    output.points.push_back(p);
  }

  sensor_msgs::PointCloud2 filtered_msg;
  pcl::toROSMsg(output, filtered_msg);

  end = std::chrono::system_clock::now();
  double time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

  ofs << msg->header.stamp << "," << time << std::endl;
  ofs.close();

  std::cout << "Duration: " << time << "\r" << std::flush;

  filtered_msg.header = msg->header;
  filtered_pub_.publish(filtered_msg);
}

void RandomFilter::run()
{
  std::cout << "points_number: " << points_number_ << std::endl;

  ros::spin();
}
