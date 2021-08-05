#include <ros/ros.h>
#include "./src/mapper.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "muin_px4");
    ROS_INFO("muin_px4, Start~ :)");
    muin_control_class * muin_controller = new muin_control_class();

    muin_controller->nh.getParam("wp_vel",muin_controller->wp_vel);
    muin_controller->nh.getParam("height_vel",muin_controller->height_vel);
    muin_controller->nh.getParam("landing_vel",muin_controller->landing_vel);
    muin_controller->nh.getParam("yaw_vel",muin_controller->yaw_vel);//rad
    muin_controller->nh.getParam("take_off_height",muin_controller->take_off_height);
    muin_controller->nh.getParam("threshold_distance",muin_controller->threshold_distance);
    muin_controller->nh.getParam("arrive_distance",muin_controller->arrive_distance);
    muin_controller->nh.getParam("threshold_yaw",muin_controller->threshold_yaw);
    muin_controller->nh.getParam("LOS_radius",muin_controller->LOS_radius);
    muin_controller->nh.getParam("auto_arrive_delay",muin_controller->auto_arrive_delay);
    muin_controller->nh.getParam("test_mode",muin_controller->test_mode);


    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
//        ROS_INFO("uav_control UAV control mode : %d",muin_controller->UAV_control_mode);
        muin_controller->spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
