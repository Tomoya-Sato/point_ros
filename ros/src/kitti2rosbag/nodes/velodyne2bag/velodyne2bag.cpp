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

  Eigen::Matrix4f Tr;

  Tr << 0.000427680238554, -0.9999672484946, -0.008084491683471, -0.01198459927713,
        -0.007210626507497, 0.008081198471645, -0.9999413164504, -0.05403984729748,
        0.9999738645903, 0.0004859485810390, -0.007206933692422, -0.2921968648686,
        0.0, 0.0, 0.0, 1.0;

  std::cout << "Transform: " << std::endl;
  std::cout << Tr << std::endl;

  char filename[64];
  for (int i = 0; i < total; i++)
  {
    sprintf(filename, "%s/%06d.bin", base_dir.c_str(), i);
    ifs.open(filename, std::ios::in | std::ios::binary);

    std::cout << "Reading: " << filename;

    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointXYZI p, q;

    while (!ifs.eof())
    {
      ifs.read((char*)&p.x, sizeof(float));
      ifs.read((char*)&p.y, sizeof(float));
      ifs.read((char*)&p.z, sizeof(float));
      ifs.read((char*)&p.intensity, sizeof(float));

      q.x = Tr(0, 0) * p.x + Tr(0, 1) * p.y + Tr(0, 2) * p.z + Tr(0, 3);
      q.y = Tr(1, 0) * p.x + Tr(1, 1) * p.y + Tr(1, 2) * p.z + Tr(1, 3);
      q.z = Tr(2, 0) * p.x + Tr(2, 1) * p.y + Tr(2, 2) * p.z + Tr(2, 3);
      q.intensity = p.intensity;
      
      output->points.push_back(q);
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
