#include <ros/ros.h>
#include "./src/control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "n_control");
    ros::NodeHandle n("~");
    ROS_INFO("N_control, Start~ :)");

    N_control _ctrl(n);

    ros::AsyncSpinner spinner(3); // Use 3 threads -> 3 callbacks
    spinner.start();
    ros::waitForShutdown();

    return 0;
}