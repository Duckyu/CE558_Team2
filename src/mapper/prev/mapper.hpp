#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <thread>
#include <sstream>
#include <queue>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

// #include <n_cpp/points_node.h>
// #include <n_cpp/points_nodes.h>
// #include <n_cpp/path_terminate.h>
// #include <n_cpp/local_path.h>
// #include <n_cpp/global_path.h>
#include <n_cpp/go_get_it.h>
#include <n_cpp/path_require.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/pcl_config.h>

#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include <tf/transform_broadcaster.h>


Eigen::Matrix4f map_t_body, body_t_front, body_t_up;
Eigen::Matrix4f front_static_trans = Eigen::Matrix4f::Identity();

pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_pc_local (new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::Normal>::Ptr ptr_pc_local_normal (new pcl::PointCloud<pcl::Normal>());

double octomap_resolution = 0.4;
octomap::OcTree *m_octree = new octomap::OcTree(octomap_resolution);
double octomap_hit = 0.65;
double octomap_miss = 0.3;
double octomap_hz = 6.0;

void setup_octomap(){
  m_octree->setProbHit(octomap_hit);
  m_octree->setProbMiss(octomap_miss);
}
