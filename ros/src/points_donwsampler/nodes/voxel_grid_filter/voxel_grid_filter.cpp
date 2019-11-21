#include "points_downsampler/voxel_grid_filter.h"

#include "voxel_grid_approxi.h"

VoxelGridFilter::VoxelGridFilter() : nh_(), private_nh_("~")
{
  private_nh_.param<double>("leafsize", leafsize_, 2.0);
  private_nh_.param<int>("method", method_, 0);

  filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_points", 10);
  points_sub_ = nh_.subscribe("points_raw", 1, &VoxelGridFilter::pointsCallback, this);
  app_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("app_points", 10);
}

VoxelGridFilter::~VoxelGridFilter()
{
}

void VoxelGridFilter::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI> input;
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr app_output(new pcl::PointCloud<pcl::PointXYZI>());

  std::ofstream ofs;
  ofs.open("voxelgrid_time.csv", std::ios::app);

  int radius_search = 0;

  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  pcl::fromROSMsg(*msg, input);
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>(input));
  
  if (method_ == 0)
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(leafsize_, leafsize_, leafsize_);
    voxel_grid_filter.setInputCloud(tmp);
    voxel_grid_filter.filter(*output);
  }  
  else if (method_ == 1)
  {
    pcl::VoxelGridApproxi<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(leafsize_, leafsize_, leafsize_);
    voxel_grid_filter.setInputCloud(tmp);
    voxel_grid_filter.filter(*output);
  }
  else
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(leafsize_, leafsize_, leafsize_);
    voxel_grid_filter.setInputCloud(tmp);
    voxel_grid_filter.filter(*output);
    
    pcl::VoxelGridApproxi<pcl::PointXYZI> voxel_grid_filter_app;
    voxel_grid_filter_app.setLeafSize(leafsize_, leafsize_, leafsize_);
    voxel_grid_filter_app.setInputCloud(tmp);
    voxel_grid_filter_app.filter(*app_output);
  }

  if (radius_search == 1)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>(input));

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);

    for (pcl::PointCloud<pcl::PointXYZI>::iterator item = output->begin(); item != output->end(); item++)
    {
      const pcl::PointXYZI& p = *item;

      std::vector<int> idx(1);
      std::vector<float> distance(1);
      if (kdtree.nearestKSearch(p, 1, idx, distance) > 0)
      {
        distance[0] = std::sqrt(distance[0]);
        item->intensity = (float)(distance[0] > 0.5);
      }
    }
  }

  sensor_msgs::PointCloud2 filtered_msg;
  pcl::toROSMsg(*output, filtered_msg);

  end = std::chrono::system_clock::now();
  double time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

  ofs << msg->header.stamp << "," << time << std::endl;
  ofs.close();

  std::cout << "Duration: " << time << "\r" << std::flush;

  filtered_msg.header = msg->header;
  filtered_pub_.publish(filtered_msg);

  if (method_ == 2)
  {
    sensor_msgs::PointCloud2 filtered_app;
    pcl::toROSMsg(*app_output, filtered_app);

    filtered_app.header = msg->header;
    app_pub_.publish(filtered_app);
  }
}

void VoxelGridFilter::run()
{
  std::cout << "leafsize : " << leafsize_ << std::endl;
  std::cout << "method   : " << method_ << std::endl;

  ros::spin();
}
