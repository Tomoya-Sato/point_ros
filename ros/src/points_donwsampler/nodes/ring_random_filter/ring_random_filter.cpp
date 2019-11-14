#include "points_downsampler/ring_random_filter.h"

RingRandomFilter::RingRandomFilter() : nh_(), private_nh_("~")
{
  private_nh_.param<int>("ray_number", ray_number_, 32);
  private_nh_.param<int>("points_number", points_nubmer_, 1500);

  filtered_pub_ = nh_.advertise<sensor_msgs::PointCLoud2>("filtered_points", 10);
  points_sub_ = nh_.subscribe("points_raw", 1, &RingRandomFilter::pointsCallback, this);
}

RingRandomFilter::~RingRandomFilter()
{
}

bool orderPoints(const pcl::PointXYZI& p0, const pcl::PointXYZI& p1)
{
  return (p1.x < p2.x);
}

bool inversePoints(const pcl::PointXYZ& p0, const pcl::PointXYZI& p1)
{
  return (p1.x > p2.x);
}

void RingRandomFilter::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> input;
  pcl::PointCloud<pcl::PointXYZI> output;

  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  pcl::fromROSMsg(*msg, input);

  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI> cloud_array(ray_number_, pcl::PointCloud<pcl::PointXYZI>(4));

  for (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator item = input.begin(); item != input.end(); item++)
  {
    pcl::PointXYZI p;
    p.x = item->x;
    p.y = item->y;
    p.z = item->z;
    p.intensity = item->intensity;

    int dim;
    if (p.x >= 0 && p.y >= 0) dim = 0;
    else if (p.x >= 0 && p.y < 0) dim = 1;
    else if (p.x < 0 && p.y <= 0) dim = 2;
    else dim = 3;

    cloud_array[item->ring][dim].points.push_back(p);
  }

  for (int i = 0; i < ray_number_; i++)
  {
    std::sort(cloud_array[i][0].begin(), cloud_array[i][0].end(), orderPoints);
    std::sort(cloud_array[i][1].begin(), cloud_array[i][1].end(), inversePoints);
    std::sort(cloud_array[i][2].begin(), cloud_array[i][2].end(), orderPoints);
    std::sort(cloud_array[i][3].begin(), cloud_array[i][3].end(), inversePoints);

    cloud_array[i][0] += cloud_array[i][1];
    cloud_array[i][0] += cloud_array[i][2];
    cloud_array[i][0] += cloud_array[i][3];
  }

  for (int i = 0; i < ray_number_; i++)
  {
    output += cloud_array[i][0];
  }

  sensor_msgs::PointCloud2 filtered_msg;
  pcl::toROSMsg(output, filtered_msg);

  end = std::chrono::system_clock::now();
  double time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

  std::cout << "Duration time: " << time << "[ms]" << "\r" << std::flush;

  filtered_msg.header = msg->header;
  filtered_pub_.publish(filtered_msg);
}

void RingRandomFilter::run();
{
  ros::spin();
}
