#include "points_downsampler/ring_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ring_filter");
  
  RingFilter node;
  node.run();

  return 0;
}
