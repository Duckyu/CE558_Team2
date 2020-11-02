#include <ros/ros.h>
#include "./src/control.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_contest");
    ROS_INFO("drone_contest, Start~ :)");
    UAV_control * UAV_controller = new UAV_control();

    UAV_controller->nh.getParam("tracker_topic",UAV_controller->tracker_topic);
    UAV_controller->nh.getParam("geofence_longitude",UAV_controller->geofence_longitude);
    UAV_controller->nh.getParam("geofence_latitude",UAV_controller->geofence_latitude);
    UAV_controller->nh.getParam("mission_height",UAV_controller->mission_height);//rad
    UAV_controller->nh.getParam("takeoff_height",UAV_controller->takeoff_height);
    UAV_controller->nh.getParam("arrival_dist_thres",UAV_controller->arrival_dist_thres);
    UAV_controller->nh.getParam("arrival_yaw_thres",UAV_controller->arrival_yaw_thres);
    UAV_controller->nh.getParam("landing_vel",UAV_controller->landing_vel);
    UAV_controller->nh.getParam("p_gain",UAV_controller->p_gain);
    UAV_controller->nh.getParam("i_gain",UAV_controller->i_gain);
    UAV_controller->nh.getParam("d_gain",UAV_controller->d_gain);
    UAV_controller->nh.getParam("img_x",UAV_controller->img_x);
    UAV_controller->nh.getParam("img_y",UAV_controller->img_y);

    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
        UAV_controller->spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
