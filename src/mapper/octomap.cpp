#include <ros/ros.h>
#include "src/octomap.hpp"
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NCPP-octomap");
    ros::NodeHandle n("~");
    ROS_INFO("NCPP-octomap, Start~ :)");
    N_octomap october(n);

    // signal(SIGINT, signal_handler);
    ros::AsyncSpinner spinner(7); // Use 7 threads -> 7 callbacks
    spinner.start();
    ros::waitForShutdown();

    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    // viewer->addCoordinateSystem(2);
    // viewer->addPointCloud(october->cloud,"original");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,1,"original");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"original");

    return 0;
}