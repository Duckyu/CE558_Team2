#include <ros/ros.h>
#include "./src/control.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "exploration_control");
    ROS_INFO("exploration control code, Start~ :)");
    exploration_control * exploration_controller = new exploration_control();

    exploration_controller->nh.getParam("takeoff_height",exploration_controller->takeoff_height);
    exploration_controller->nh.getParam("arrival_dist_thres",exploration_controller->arrival_dist_thres);
    exploration_controller->nh.getParam("arrival_yaw_thres",exploration_controller->arrival_yaw_thres);
    exploration_controller->nh.getParam("max_vel",exploration_controller->max_vel);
    exploration_controller->nh.getParam("landing_vel",exploration_controller->landing_vel);
    exploration_controller->nh.getParam("p_gain",exploration_controller->p_gain);
    exploration_controller->nh.getParam("i_gain",exploration_controller->i_gain);
    exploration_controller->nh.getParam("d_gain",exploration_controller->d_gain);
    exploration_controller->nh.getParam("img_x",exploration_controller->img_x);
    exploration_controller->nh.getParam("img_y",exploration_controller->img_y);

    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
        exploration_controller->spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
