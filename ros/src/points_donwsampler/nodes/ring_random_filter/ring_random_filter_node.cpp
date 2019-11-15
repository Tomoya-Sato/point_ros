#include "points_downsampler/ring_random_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ring_random_filter");

    RingRandomFilter node;
    node.run();

    return 0;
}