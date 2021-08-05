
#include <ros/ros.h>
#include "normal_path_sampler/normal_path_sampler.hpp"
#include "local_pc_registration/local_pc_registration.hpp"

int main(int argc, char **argv)
{


    ros::init(argc, argv, "path_gen");
    ROS_INFO("exploration path_gen code, Start~ :)");
    exploration_local_map * exploration_local_mapper = new exploration_local_map();
    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
        exploration_local_mapper->spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
