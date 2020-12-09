
#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <sstream>

#include <tf/transform_listener.h>

#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <CE558_Team2/points_node.h>
#include <CE558_Team2/points_nodes.h>
#include <CE558_Team2/path_terminate.h>
#include <CE558_Team2/local_path.h>
#include <CE558_Team2/global_path.h>

#include <pcl/conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <unavlib/convt.h>
#include <unavlib/others.h>

class exploration_local_map
{
private:
  //sub
  ros::Subscriber sub_front_points;
  ros::Subscriber sub_up_points;
  ros::Subscriber sub_model_state;

  ros::ServiceClient srv_local_path;
  ros::ServiceClient srv_global_path;

  void cb_points_front(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void cb_points_up(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void cb_model_state(const geometry_msgs::PoseStamped::ConstPtr& msg);
  // ros::Subscriber sub_octoMap;

  ros::ServiceServer srv_path_termination;
  bool fn_path_terminate(CE558_Team2::path_terminate::Request &request,
                         CE558_Team2::path_terminate::Response &response);


  // octomap::OcTree * m_octree;

  // Eigen::Matrix4f m_tf_robot2sensor;
  // Eigen::Matrix4f m_tf_sensor2lidar;
  // float m_resolution;

  // void getparam();

  // pcl::PCLPointCloud2 pcl_pc2;
  // pcl_conversions::toPCL(*input,pcl_pc2);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

public:

  float horiz_fov = 1.02974;

  ros::NodeHandle m_nh;/**< Ros node handler */

  CE558_Team2::points_node up_;
  CE558_Team2::points_node front_;


  CE558_Team2::points_nodes previous;
  pcl::PointCloud<pcl::PointXYZ> previous_pc_accumulated;
  pcl::PointCloud<pcl::PointXYZ> pc_accumulated;


  pcl::PointCloud<pcl::PointXYZ> pc_transformed;
  pcl::PointCloud<pcl::PointNormal> normal_points;
  pcl::PointCloud<pcl::PointNormal> normal_voxeled_points;
  nav_msgs::Path local_path;
  geometry_msgs::PoseStamped pose_path[];

  ros::Time last_pc_added;
  geometry_msgs::PoseStamped pose_previous, pose_current;
  bool termination_flag;

  CE558_Team2::points_node msg2node(sensor_msgs::PointCloud2 points,
                                    geometry_msgs::PoseStamped pose_previous,
                                    geometry_msgs::PoseStamped pose_current);
  pcl::PointCloud<pcl::PointXYZ> transform(CE558_Team2::points_node node);
  // void node2pcl(CE558_Team2::points_node node);
  void rangeFiltering(CE558_Team2::points_node node,
                      int axis, double location_low, double location_high);
  void add(CE558_Team2::points_node node);
  void voxelize(pcl::PointCloud<pcl::PointXYZ> pc);
  void normalVectorEstimate(pcl::PointCloud<pcl::PointXYZ> pc);
  void samplingBasePathGen();
  // void checkCollision(geometry_msgs::PoseStamped poses[], octomap::OcTree octomap);
  void pubPath(geometry_msgs::PoseStamped poses[]);

  /// For main function
  void spinOnce();

  localMap(){
    sub_front_points = nh.subscribe<nav_msgs::Odometry>("/camera_front/depth/points", 1, &exploration_local_map::cb_points_front,this);
    sub_up_points = nh.subscribe<nav_msgs::Odometry>("/camera_up/depth/points", 1, &exploration_local_map::cb_points_up,this);
    sub_model = nh.subscribe<geometry_msgs::PoseStamped>("/UAV", 1, &exploration_local_map::cb_model_state,this);

    srv_local_path = nh.serviceClient<CE558_Team2::local_path>("/exploration_control/local_path");
    srv_global_path = nh.serviceClient<CE558_Team2::global_path>("/exploration_control/global_path");

    srv_path_termination = nh.advertiseService("/exploration_control/path_terminate", &exploration_local_map::fn_path_terminate, this);
  }
  ~localMap();


};

#endif

