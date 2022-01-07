#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <random>

//transformation
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//ros msg
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Duration.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//pcl 
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_config.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//Faster library NanoFLANN
#include "nanoflann_pcl.h"

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

//mpl ellipsoid planner
#include <decomp_ros_utils/data_ros_utils.h>
#include <mpl_external_planner/ellipsoid_planner/ellipsoid_planner.h>
// #include <planning_ros_utils/data_ros_utils.h>
// #include <planning_ros_utils/primitive_ros_utils.h>
//custom
#include "n_cpp/NextStep.h"
#include "n_cpp/RayCast.h"
// #include "n_cpp/ThreeDAstar.h"
#include "n_cpp/get_octomap.h"
#include "n_cpp/mission_terminate.h"
#include "n_cpp/collision_check.h"
#include "n_cpp/get_triple_points.h"

class N_path
{
private:
	ros::ServiceServer srv_next_step;
	ros::ServiceServer srv_mission_terminate;

	ros::ServiceClient client_get_octomap;
	ros::ServiceClient client_triplepoints;
	ros::ServiceClient client_collision_check;
	ros::ServiceClient client_ray_cast;

	// ros::ServiceClient client_astar;

	ros::Publisher normal_pub;
	ros::Publisher closest_tri_pub;
	ros::Publisher cand_pub;
	ros::Publisher path_pub;
	ros::Publisher feature_traj_pub;
	ros::Publisher raw_traj_pub;

	ros::Subscriber pix_geoLocal_sub;
	ros::Subscriber octo_occu_sub;

	geometry_msgs::PoseStamped curr_pose;
	sensor_msgs::PointCloud2 octo_new;
	sensor_msgs::PointCloud2 octo_occu;
	sensor_msgs::PointCloud2 tripoints;
	geometry_msgs::PoseStamped octo_pose;

	std::vector<geometry_msgs::Point> remain_ref_points;
	nav_msgs::Path candidate;
	std::vector<geometry_msgs::Vector3> candidate_normals;
	// nav_msgs::Path local_path;

    void subs_pix_local_geo(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void subs_octo_occu(const sensor_msgs::PointCloud2::ConstPtr &msg);

    bool next_step(n_cpp::NextStep::Request &req,
                   n_cpp::NextStep::Response &res);
	// bool Astar(n_cpp::ThreeDAstar::Request &req,
 //               n_cpp::ThreeDAstar::Response &res);
    
    bool mission_terminate(n_cpp::mission_terminate::Request &req,
                           n_cpp::mission_terminate::Response &res);

	double offset_distance;
	bool correction;
	double ground_thresh;
	double down_size;
	int    num_knn;
	double path_min;
	double path_max;
	double octomap_resolution;
	double dist_ratio;
	int min_cluster;
	int max_cluster;

	double robot_radius;
	double dt, v_max, a_max, yaw_max, u_max, u_yaw, w, epsilon;
	int max_num, num;
	
	bool pub_octomap=true;
	bool pub_candidate;

	sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id = "camera_link");
	pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg);
	octomap::Pointcloud cloudmsg2octocloud(sensor_msgs::PointCloud2 cloudmsg);
	octomap::Pointcloud cloud2octocloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn);
	octomap::Pointcloud cloud2transOctoCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Matrix4f map_t_sensor);

	octomath::Pose6D cvt2octomath(nav_msgs::Odometry curr_pose);
	octomath::Pose6D cvt2octomath(geometry_msgs::PoseStamped curr_pose);

	// Eigen::Vec3f pose2eigen(geometry_msgs::PoseStamped pose);
	// Eigen::Vec3f pose2eigen(geometry_msgs::Pose pose);

	float normalize(float a, float b);
	float normalize(float a, float b, float c);
	float normalize(float a, float b, float c, float d);
	double normalize(double a, double b);
	double normalize(double a, double b, double c);
	double normalize(double a, double b, double c, double d);


public:
	ros::NodeHandle nh;

	ros::Time init_time;

	size_t marker_num =0;

	visualization_msgs::MarkerArray normals_marker;

	nav_msgs::Path path_gen();
	// nav_msgs::Path PathToStart(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end);
	// planning_ros_msgs::Trajectory PathToStart(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end);
	n_cpp::NextStep::Response new_path_gen(bool escape);
	
	N_path(ros::NodeHandle& n) : nh(n)
	{
		
		nh.param("robot_r", robot_radius, 0.5);
		nh.param("dt", dt, 1.0);
		nh.param("epsilon", epsilon, 1.0);
		nh.param("v_max", v_max, -1.0);
		nh.param("a_max", a_max, -1.0);
		nh.param("u_max", u_max, 1.0);
		nh.param("u_yaw", u_yaw, 1.0);
		nh.param("yaw_max", yaw_max, -1.0);
		nh.param("w", w, 10.);
		nh.param("num", num, 1);
		nh.param("max_num", max_num, -1);

		nh.param("/correction", correction, false);
		nh.param("/offset_distance", offset_distance, 1.5);
		nh.param("/ground_thresh", ground_thresh, 0.05);
		nh.param("/down_size", down_size, 1.0);
		nh.param("/num_knn", num_knn, 30);
		nh.param("/path_min", path_min, 0.5);
		nh.param("/path_max", path_max, 0.5);

		
		nh.param("/octomap_resolution", octomap_resolution, 0.15);
		nh.param("/dist_ratio", dist_ratio, 2.0);
		nh.param("/min_cluster", min_cluster, 60);
		nh.param("/max_cluster", max_cluster, 4);
		nh.param("/pub_octomap", pub_octomap, false);
		nh.param("/pub_candidate", pub_candidate, false);

		srv_next_step = nh.advertiseService("next_step", &N_path::next_step, this);
		srv_mission_terminate = nh.advertiseService("mission_terminate", &N_path::mission_terminate, this);

		pix_geoLocal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &N_path::subs_pix_local_geo, this, ros::TransportHints().tcpNoDelay());
		octo_occu_sub = nh.subscribe<sensor_msgs::PointCloud2>("/N_octo/occu", 1, &N_path::subs_octo_occu, this, ros::TransportHints().tcpNoDelay());

		client_get_octomap = nh.serviceClient<n_cpp::get_octomap>("/n_octomap/get_octomap");
		client_triplepoints = nh.serviceClient<n_cpp::get_triple_points>("/n_octomap/get_triple_points");
		client_collision_check = nh.serviceClient<n_cpp::collision_check>("/n_octomap/collision_check");
		client_ray_cast = nh.serviceClient<n_cpp::RayCast>("/n_octomap/ray_cast");
		// client_astar = nh.serviceClient<n_cpp::ThreeDAstar>("/n_octomap/three_astar");
        
		normal_pub = nh.advertise<visualization_msgs::MarkerArray>("/n_path/normal", 10, this);
		closest_tri_pub = nh.advertise<sensor_msgs::PointCloud2>("/n_path/closest_tri", 10, this);
		cand_pub = nh.advertise<visualization_msgs::MarkerArray>("/n_path/cand", 10, this);
		path_pub = nh.advertise<nav_msgs::Path>("/n_path/raw_path", 10, this);
		raw_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("/n_path/raw_traj", 10, this);
		feature_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("/n_path/feature_traj", 10, this);
	};
};

