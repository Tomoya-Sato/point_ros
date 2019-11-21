#include "points_downsampler/random_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "random_filter");

  RandomFilter node;
  node.run();

  return 0;
}
