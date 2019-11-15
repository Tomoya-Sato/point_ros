#include "points_downsampler/distance_voxel_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_voxel_filter");

  DistanceVoxelFilter node;
  node.run();

  return 0;
}