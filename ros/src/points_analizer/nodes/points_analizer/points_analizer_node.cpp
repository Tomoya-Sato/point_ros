#include "points_analizer/points_analizer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_analizer");
  PointsAnalizer node;
  node.run();

  return 0;
}
