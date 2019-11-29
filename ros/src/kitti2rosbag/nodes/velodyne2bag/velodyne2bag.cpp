#include <iostream>
#include <fstream>
#include <string>

#include <rosbag/bag.h>

#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne2rosbag");
  ros::NodeHandle nh;

  std::string base_dir(argv[1]);
  int total = atoi(argv[2]);
  std::string output(argv[3]);
  std::ifstream ifs;

  ros::Time base;
  base = ros::Time::now();

  ros::Duration time_step(0.1);

  rosbag::Bag out_bag;
  out_bag.open(output, rosbag::bagmode::Write);

  char filename[64];
  for (int i = 0; i < total; i++)
  {
    sprintf(filename, "%s/%06d.bin", base_dir.c_str(), i);
    ifs.open(filename, std::ios::in | std::ios::binary);

    std::cout << "Reading: " << filename;

    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointXYZI p;

    while (!ifs.eof())
    {
      ifs.read((char*)&p.x, sizeof(float));
      ifs.read((char*)&p.y, sizeof(float));
      ifs.read((char*)&p.z, sizeof(float));
      ifs.read((char*)&p.intensity, sizeof(float));

      output->points.push_back(p);
    }

    std::cout << ", points_size: " << output->size() << std::endl;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*output, msg);
    msg.header.stamp = base;
    msg.header.frame_id = "velodyne";
    msg.header.seq = i;

    out_bag.write("/points_raw", base, msg);

    base = base + time_step;
    ifs.close();
    output->clear();
  }

  out_bag.close();

  return 0;
}
