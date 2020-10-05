
#include <ros/ros.h>
#include "src/octoMapgen.h"

using namespace imSLAM;

int main(int argc, char **argv)
{


    ros::init(argc, argv, "im_octoMapgen");

    octoMapgen octoMapgen_class = octoMapgen();
    ros::spin();

    return 0;
}
