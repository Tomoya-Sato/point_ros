#include "points_downsampler/ring_random_filter.h"

RingRandomFilter::RingRandomFilter() : nh_(), private_nh_("~")
{
  private_nh_.param<int>("ray_number", ray_number_, 32);
  private_nh_.param<int>("points_number", points_number_, 1500);

  filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_points", 10);
  points_sub_ = nh_.subscribe("points_raw", 1, &RingRandomFilter::pointsCallback, this);
}

RingRandomFilter::~RingRandomFilter()
{
}

void RingRandomFilter::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> input;
  pcl::PointCloud<pcl::PointXYZI> output;

  std::ofstream ofs;
  ofs.open("ring_random_time.csv", std::ios::app);

  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  pcl::fromROSMsg(*msg, input);

  int stride = input.size() / points_number_;

  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>>> cloud_array(ray_number_, std::vector<pcl::PointCloud<pcl::PointXYZI>>(4));

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
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = cloud_array[i][0].begin(); item < cloud_array[i][0].end(); item+=stride)
    {
      pcl::PointXYZI q;
      q.x = item->x;
      q.y = item->y;
      q.z = item->z;
      q.intensity = item->intensity;

      output.points.push_back(q);
    }
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

void RingRandomFilter::run()
{
  std::cout << "ray_number   : " << ray_number_ << std::endl;
  std::cout << "points_number: " << points_number_ << std::endl;
  ros::spin();
}
