#include "points_analizer/points_analizer.h"

PointsAnalizer::PointsAnalizer() : nh_(), private_nh_("~")
{
  private_nh_.param<std::string>("log_name", log_name_, "points_analizer_log.csv");
  private_nh_.param<std::string>("points_name", points_name_, "filtered_points");

  points_sub_ = nh_.subscribe(points_name_.c_str(), 1, &PointsAnalizer::pointsCallback, this);
  end_sub_ = nh_.subscribe("end_sign", 1, &PointsAnalizer::endCallback, this);
}

PointsAnalizer::~PointsAnalizer()
{
}

void PointsAnalizer::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> input;
  pcl::fromROSMsg(*msg, input);

  std::ofstream dis_ofs;
  dis_ofs.open("distance_distribution.csv", std::ios::app);

  std::vector<int> distribution(20, 0);

  double distance_sum = 0.0;
  int points_num = 0;

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = input.begin(); item != input.end(); item++)
  {
    double distance = std::sqrt(item->x * item->x + item->y * item->y + item->z * item->z);
    distance_sum += distance;
    points_num++;

    if (distance < 200)
    {
      distribution[distance / 10] += 1
    }
  }

  double average_distance = distance_sum / points_num;

  ofs_ << msg->header.stamp << "," << average_distance << "," << points_num << std::endl;
  for (int i = 0; i < 20; i++)
  {
    dis_ofs << distribution[i] << ",";
  }
  dis_ofs << std::endl;
  dis_ofs << close();
}

void PointsAnalizer::endCallback(const std_msgs::Bool msg)
{
  ofs_.close();
  exit(0);
}

void PointsAnalizer::run()
{
  ofs_.open(log_name_.c_str());
  if (!ofs_.is_open())
  {
    std::cerr << "Could not open: " << log_name_ << std::endl;
    exit(EXIT_FAILURE);
  }

  ros::spin();
}
