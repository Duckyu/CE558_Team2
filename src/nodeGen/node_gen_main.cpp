#include <ros/ros.h>
#include "./src/node_gen.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_gen");
    ROS_INFO("node_gen, Start~ :)");
    node_gen * node_gen = new node_gen();
    std::string robot2cam;
    node_gen->nh.getParam("robot2cam", robot2cam);
    node_gen->nh.getParam("diff_distance;",node_gen->diff_distance;);
    node_gen->nh.getParam("diff_time;",node_gen->diff_time;);
    node_gen->nh.getParam("cov_last2curr;",node_gen->cov_last2curr;);
    node_gen->nh.getParam("time_offset",node_gen->time_offset);
    node_gen->nh.getParam("test_mode",node_gen->test_mode);

    node_gen->translate2mat(robot2cam,node_gen->mat_robot2cam);

    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
//        ROS_INFO("test mode : %d",node_gen->test_mode);
        node_gen->spinOnce();
        loop_rate.sleep();
    }
    return 0;
}