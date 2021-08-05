#include <ros/ros.h>
#include "./src/path_gen.hpp"
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NCPP-pathgen");
    ros::NodeHandle n("~");
    ROS_INFO("NCPP-pathgen, Start~ :)");
    N_path panther(n);
    panther.init_time = ros::Time::now();

    // signal(SIGINT, signal_handler);
    ros::AsyncSpinner spinner(5); // Use 4 threads -> 4 callbacks
    spinner.start();
    ros::waitForShutdown();


    return 0;
}