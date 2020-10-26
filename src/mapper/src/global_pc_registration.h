/** @file splitter.h
    @date 2018/12
    @author Seungwon Song
    @brief  Generate node which contain sensor data
*/

#ifndef OCTOMAPGEN_H
#define OCTOMAPGEN_H

#include <ros/ros.h>
#include <sstream>

#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/filters/uniform_sampling.h>

#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include "imSLAM/im_node.h"
#include "unavlib/convt.h"
#include "unavlib/others.h"

namespace imSLAM
{
class octoMapgen
{
private:

  ros::NodeHandle m_nh;/**< Ros node handler */
  ros::Publisher m_pub_octoMap;

  octomap::OcTree * m_octree;

  Eigen::Matrix4f m_tf_robot2sensor;
  Eigen::Matrix4f m_tf_sensor2lidar;
  float m_resolution;

  void getparam();

  void callback_node(const imSLAM::im_node::ConstPtr& msg);
  void callback_octoPub(const std_msgs::Int32::ConstPtr &msg);


public:
  octoMapgen();
  ~octoMapgen();


};

}


#endif

