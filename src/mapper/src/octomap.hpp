#define FRONT 0
#define UP 1
#define RIGHT 2
//sensor_direction

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <math.h>

//transformation
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//ros msg
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

//pcl 
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

//additional
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//octomap lib
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include <octomap/ScanGraph.h>

//custom service
#include "n_cpp/get_octomap.h"
#include "n_cpp/get_triple_points.h"
#include "n_cpp/collision_check.h"
#include "n_cpp/RayCast.h"
#include "n_cpp/EliminateTriple.h"
// #include "n_cpp/ThreeDAstar.h"
// #include "n_cpp/FindCollisionAvoidPath.h"

class N_octomap
{
private:

    ros::ServiceServer srv_get_octomap;
    ros::ServiceServer srv_get_triple_points;
    ros::ServiceServer srv_ray_cast;
    ros::ServiceServer srv_eliminate_tri;
    // ros::ServiceServer srv_astar_planner;
    // ros::ServiceServer srv_find_collision_free_path;
    ros::ServiceServer srv_collision_check;

	ros::Publisher octo_occu_pub;
	ros::Publisher octo_free_pub;
	ros::Publisher octo_change_pub;
	ros::Publisher tri_points_pub;

	ros::Subscriber sub_points_front;
	ros::Subscriber sub_points_up;
	ros::Subscriber sub_pose;
	// ros::Subscriber sub_tf;
	ros::Subscriber sub_st_tf;

	ros::Timer octo_input_timer;

	// pcl::PointIndices::Ptr triple_point_idx (new pcl::PointIndices ());
	// pcl::PointCloud<pcl::PointXYZ>::Ptr get_triple_points(new pcl::PointCloud<pcl::PointXYZ>());
	std::vector<octomap::OcTreeKey> triple_ban_list;
	std::vector<octomap::OcTreeKey> triple_points;
	geometry_msgs::PoseStamped curr_pose;
	sensor_msgs::PointCloud2 _front, _up;
	bool detection_flag;

	Eigen::Matrix4f map_t_body = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f body_t_front = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f body_t_up = Eigen::Matrix4f::Identity();
	// Eigen::Matrix4f map_t_front;
	// Eigen::Matrix4f map_t_up;
	double sensor_range;
	double octomap_resolution;
	double octomap_hit;
	double octomap_miss;
	double octomap_hz;

	int num_neigbor_points;
  double std_multiplier;
  int valid_min_num;
  double ground_thresh;
  double looking_sky;

	octomap::OcTree *m_octree;

	sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id = "camera_link");
	pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg);
	octomap::Pointcloud cloudmsg2octocloud(sensor_msgs::PointCloud2 cloudmsg);
	octomap::Pointcloud cloud2octocloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn);
	octomap::Pointcloud cloud2transOctoCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Matrix4f map_t_sensor);

	octomath::Pose6D cvt2octomath(nav_msgs::Odometry curr_pose);
	octomath::Pose6D cvt2octomath(geometry_msgs::PoseStamped curr_pose);

	bool is_this_triplepoint(octomap::OcTree *octree, octomap::OcTreeKey tripoint_candidate);

	bool data_input(octomap::OcTree *octree, sensor_msgs::PointCloud2 input, int sensor_direction);
	
	void subs_front(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void subs_up(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void subs_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	// void cb_tf(const tf2_msgs::TFMessage::ConstPtr& msg);
	void cb_st_tf(const tf2_msgs::TFMessage::ConstPtr& msg);

	void cvt_octo2pubpc(octomap::OcTree *octree);

	bool get_changed_octomap(n_cpp::get_octomap::Request &req,
                        	 n_cpp::get_octomap::Response &res);
	bool get_triple_points(n_cpp::get_triple_points::Request &req,
                           n_cpp::get_triple_points::Response &res);
    bool ray_cast(n_cpp::RayCast::Request &req,
                  n_cpp::RayCast::Response &res);
    bool eliminate_tri(n_cpp::EliminateTriple::Request &req,
                       n_cpp::EliminateTriple::Response &res);
    // bool astar_planner(n_cpp::ThreeDAstar::Request &req,
    //                    n_cpp::ThreeDAstar::Response &res);
    // bool find_collision_free_path(n_cpp::FindCollisionAvoidPath::Request &req,
    //                    						n_cpp::FindCollisionAvoidPath::Response &res);

	bool candidate_check(n_cpp::collision_check::Request &req,
                         n_cpp::collision_check::Response &res);

	float normalize(float a, float b);
	float normalize(float a, float b, float c);
	float normalize(float a, float b, float c, float d);
	double normalize(double a, double b);
	double normalize(double a, double b, double c);
	double normalize(double a, double b, double c, double d);

public:
	ros::NodeHandle nh;

	void input_timer(const ros::TimerEvent& event);
	
	N_octomap(ros::NodeHandle& n) : nh(n)
	{
		nh.param("/sensor_range", sensor_range, 18.0);
    nh.param("/octomap_resolution", octomap_resolution, 0.4);
    nh.param("/octomap_hit", octomap_hit, 0.65);
    nh.param("/octomap_miss", octomap_miss, 0.3);
    nh.param("/octomap_hz", octomap_hz, 6.0);
    nh.param("/num_neigbor_points", num_neigbor_points, 10);
    nh.param("/std_multiplier", std_multiplier, 1.0);
    nh.param("/valid_min_num", valid_min_num, 1);
    nh.param("/ground_thresh", ground_thresh, 0.5);
    nh.param("/looking_sky", looking_sky, 3.0);

		m_octree = new octomap::OcTree(octomap_resolution);
		m_octree -> setProbHit(octomap_hit);
	 	m_octree -> setProbMiss(octomap_miss);
	 	m_octree -> enableChangeDetection(true);

		octo_occu_pub    = nh.advertise<sensor_msgs::PointCloud2>("/N_octo/occu", 10);
		octo_free_pub    = nh.advertise<sensor_msgs::PointCloud2>("/N_octo/free", 10);
		octo_change_pub  = nh.advertise<sensor_msgs::PointCloud2>("/N_octo/new", 10);
		tri_points_pub  = nh.advertise<sensor_msgs::PointCloud2>("/N_octo/tri_points", 10);

		sub_points_front = nh.subscribe<sensor_msgs::PointCloud2>("/camera_front/depth/points", 1, &N_octomap::subs_front,this, ros::TransportHints().tcpNoDelay());
		sub_points_up    = nh.subscribe<sensor_msgs::PointCloud2>("/camera_up/depth/points", 1, &N_octomap::subs_up,this, ros::TransportHints().tcpNoDelay());
		sub_pose         = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &N_octomap::subs_pose,this, ros::TransportHints().tcpNoDelay());

		// sub_tf = nh.subscribe("/tf", 1000, &N_octomap::cb_tf, this, ros::TransportHints().tcpNoDelay());
	    sub_st_tf = nh.subscribe("/tf_static", 1000, &N_octomap::cb_st_tf, this, ros::TransportHints().tcpNoDelay());

    	srv_get_octomap                = nh.advertiseService("get_octomap", &N_octomap::get_changed_octomap, this);
    	srv_get_triple_points          = nh.advertiseService("get_triple_points", &N_octomap::get_triple_points, this);
    	srv_ray_cast                   = nh.advertiseService("ray_cast", &N_octomap::ray_cast, this);
			srv_eliminate_tri              = nh.advertiseService("eliminate_tri", &N_octomap::eliminate_tri, this);
			// srv_astar_planner              = nh.advertiseService("three_astar", &N_octomap::astar_planner, this);		
			// srv_find_collision_free_path   = nh.advertiseService("find_collision_free_path", &N_octomap::find_collision_free_path, this); // FindCollisionAvoidPath		


    	srv_collision_check       = nh.advertiseService("collision_check", &N_octomap::candidate_check, this);

		octo_input_timer    = nh.createTimer(ros::Duration(1/octomap_hz), &N_octomap::input_timer, this); // every hz
	};
};