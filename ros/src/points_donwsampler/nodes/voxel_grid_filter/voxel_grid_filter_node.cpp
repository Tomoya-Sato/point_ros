#include "points_downsampler/voxel_grid_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "voxel_grid_filter");

  VoxelGridFilter node;
  node.run();

  return 0;
}
