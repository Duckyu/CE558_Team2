#ifndef OCTOMAPGEN_H
#define OCTOMAPGEN_H

#include <ros/ros.h>
#include <sstream>

#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/filters/uniform_sampling.h>

#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include "unavlib/convt.h"
#include "unavlib/others.h"

namespace duck
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

  void callback_pose(const imSLAM::im_node::ConstPtr& msg);
  void callback_octoPub(const std_msgs::Int32::ConstPtr &msg);


public:
  octoMapgen()
  {
    //  static ros::Subscriber sub1 = m_nh.subscribe<imSLAM::de_node>("deepExpress/octoMapgen/nodeIn",1,&octoMapgen::callback_node,this);
    static ros::Subscriber sub1 = m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10000,&octoMapgen::callback_pose,this);
    m_pub_octoMap = m_nh.advertise<sensor_msgs::PointCloud2>("imSLAM/octoMapgen/debug/octoMap",100);
    static ros::Subscriber sub4 = m_nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",1000,&octoMapgen::callback_octoPub,this);

    getparam();

    m_octree = new octomap::OcTree(m_resolution);
    m_octree->setProbHit(0.6);
    m_octree->setProbMiss(0.4);
  };
  ~octoMapgen();
};

}


#endif