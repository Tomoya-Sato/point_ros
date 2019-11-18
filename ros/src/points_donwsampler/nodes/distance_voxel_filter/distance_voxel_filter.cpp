#include "points_downsampler/distance_voxel_filter.h"

// #include "voxel_grid_approxi.h"

DistanceVoxelFilter::DistanceVoxelFilter() : nh_(), private_nh_("~")
{
  private_nh_.param<double>("max_range", max_range_, 200);
  private_nh_.param<double>("near_leafsize", near_leafsize_, 1.0);
  private_nh_.param<double>("dist_leafsize", dist_leafsize_, 3.0);
  
  filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_points", 10);
  points_sub_ = nh_.subscribe("points_raw", 1, &DistanceVoxelFilter::pointsCallback, this);
}

DistanceVoxelFilter::~DistanceVoxelFilter()
{
}

void DistanceVoxelFilter::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI> input;
  pcl::PointCloud<pcl::PointXYZI> output;

  std::ofstream ofs;
  ofs.open("filter_time.csv", std::ios::app);

  std::chrono::time_point<std::chrono::system_clock> start, end, a, b;
  start = std::chrono::system_clock::now();

  pcl::fromROSMsg(*msg, input);

  pcl::PointCloud<pcl::PointXYZI>::Ptr near(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr dist(new pcl::PointCloud<pcl::PointXYZI>());

  double max_r = max_range_ * max_range_;
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = input.begin(); item != input.end(); item++)
  {
    const pcl::PointXYZI& p = *item;

    if ((p.x * p.x + p.y * p.y + p.z * p.z) < max_r)
    {
      near->points.push_back(p);
    }
    else
    {
      dist->points.push_back(p);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr near2(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr dist2(new pcl::PointCloud<pcl::PointXYZI>());

  voxel_grid_filter_.setLeafSize(near_leafsize_, near_leafsize_, near_leafsize_);
  voxel_grid_filter_.setInputCloud(near);
  voxel_grid_filter_.filter(*near2);

  voxel_grid_filter_.setLeafSize(dist_leafsize_, dist_leafsize_, dist_leafsize_);
  voxel_grid_filter_.setInputCloud(dist);
  voxel_grid_filter_.filter(*dist2);

  output += *near2;
  output += *dist2;
  
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

void DistanceVoxelFilter::run()
{
  std::cout << "max_range    : " << max_range_ << std::endl;
  std::cout << "near_leafsize: " << near_leafsize_ << std::endl;
  std::cout << "dist_leafsize: " << dist_leafsize_ << std::endl;

  ros::spin();
}
