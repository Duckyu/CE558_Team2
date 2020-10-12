
#include <ros/ros.h>
#include "src/octoMapgen.h"

int main(int argc, char **argv)
{


    ros::init(argc, argv, "global_octoMapgen");

    octoMapgen octoMapgen_class = octoMapgen();
    ros::spin();

    return 0;
}
