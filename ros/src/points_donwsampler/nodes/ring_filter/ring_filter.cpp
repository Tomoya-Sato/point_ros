#include "points_downsampler/ring_filter.h"

RingFilter::RingFilter() : nh_(), private_nh_("~")
{
  private_nh_.param<int>("ray_number", ray_number_, 32);
  private_nh_.param<float>("max_leaf_size", max_leafsize_, 2.0);
  private_nh_.param<float>("min_leaf_size", min_leafsize_, 0.5);

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

  std::ofstream ofs;
  ofs.open("ring_filter_time.csv", std::ios::app);

  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  pcl::fromROSMsg(*msg, input);

  std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_array(ray_number_);

  for (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator item = input.begin(); item != input.end(); item++)
  {
    pcl::PointXYZI p;
    p.x = item->x;
    p.y = item->y;
    p.z = item->z;
    p.intensity = item->intensity;

    cloud_array[item->ring].points.push_back(p);
  }

  float leafsize = min_leafsize_;
  float leaf_interval = (max_leafsize_ - min_leafsize_) / (ray_number_ - 1);

  for (int i = 0; i < ray_number_; i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud_array[i]));
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>());

    voxel_grid_filter_.setLeafSize(leafsize, leafsize, leafsize);
    voxel_grid_filter_.setInputCloud(input_ptr);
    voxel_grid_filter_.filter(*filtered_ptr);

    output += *filtered_ptr;
    leafsize += leaf_interval;
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

void RingFilter::run()
{
  std::cout << "max_leaf_size: " << max_leafsize_ << std::endl;
  std::cout << "min_leaf_size: " << min_leafsize_ << std::endl;
  std::cout << "ray_number   : " << ray_number_ << std::endl;

  ros::spin();
}
