#include <iostream>
#include <fstream>
#include <string>

#include <rosbag/bag.h>

#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne2bag");
  ros::NodeHandle n;

  if (argc != 5)
  {
    std::cerr << "Invalid Arguments!" << std::endl;
    std::cerr << "velodyne2bag $DATA_BASE_DIR $FILE_NUM $STAMP_FILE $OUTPUT_BAG" << std::endl;

    return -1;
  }

  std::string base_dir(argv[1]);
  int total = atoi(argv[2]);
  std::string stamp_file(argv[3]);
  std::string bag_name(argv[4]);
  std::ifstream data_ifs;
  std::ifstream stamp_ifs;

  std::cout << "############" << std::endl
            << "## Config ##" << std::endl
            << "############" << std::endl;

  std::cout << "DATA BASE DIR: " << base_dir << std::endl;
  std::cout << "TOTAL FILE NUBMER: " << total << std::endl;
  std::cout << "STAMP FILE: " << stamp_file << std::endl;
  std::cout << "OUTPUT BAG FILE: " << bag_name << std::endl;
  std::cout << std::endl;

  rosbag::Bag out_bag;
  out_bag.open(bag_name, rosbag::bagmode::Write);

  stamp_ifs.open(stamp_file, std::ios::in);

  ros::Time base_stamp = ros::Time::now();

  char filename[1024];
  for (int i = 0; i < total; i++)
  {
    sprintf(filename, "%s/%06d.bin", base_dir.c_str(), i);
    data_ifs.open(filename, std::ios::in | std::ios::binary);

    std::cout << "Reading: " << filename;

    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointXYZI p;

    while (!data_ifs.eof())
    {
      data_ifs.read((char*)&p.x, sizeof(float));
      data_ifs.read((char*)&p.y, sizeof(float));
      data_ifs.read((char*)&p.z, sizeof(float));
      data_ifs.read((char*)&p.intensity, sizeof(float));

      output->points.push_back(p);
    }

    std::cout << ", points_size: " << output->size();

    std::string stamp_tmp;
    std::getline(stamp_ifs, stamp_tmp);
    double d_stamp = std::stod(stamp_tmp);
    ros::Duration stamp_duration(d_stamp);
    ros::Time ros_stamp = base_stamp + stamp_duration;

    std::cout << " [" << d_stamp << "]" << std::endl;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*output, msg);
    msg.header.stamp = ros_stamp;
    msg.header.frame_id = "velodyne";
    msg.header.seq = i;

    out_bag.write("/points_raw", ros_stamp, msg);

    data_ifs.close();
    output->clear();
  }

  out_bag.close();

  return 0;
}
