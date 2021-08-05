#include "../../general/general.h"

#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
//#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <n_cpp/point_node.h>
#include <n_cpp/point_nodes.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

class node_gen
{
private:
    ros::NodeHandle nh_priv;
    n_cpp::point_node last_node;


    bool condition_distance();
    bool condition_time();
    bool condition_cov();    
    void node_gen_trigger(bool condition);

    // ros::ServiceServer server_global_ex;
    // ros::Subscriber sub_marker_passed_path;

    void interpolate();

    void cb_pose(const geometry_msgs::PoseStamped::ConstPtr &pose);
    void cb_pointcloud(const sensor_msgs::PointCloud2::ConstPtr &sweep);

    bool fn_mission_start(drone_contest::mission_start::Request &request,
                          drone_contest::mission_start::Response &response);
    
public:
    ros::NodeHandle nh;

    float diff_distance;
    float diff_time;
    float cov_last2curr;
    float time_offset;
    int test_mode;

    Eigen::Affine3f mat_robot2cam;

    void translate2mat(std::string str, Eigen::Affine3f &mat);
    
    void spinOnce();

    node_gen();
    ~node_gen();
};

node_gen::node_gen()
{
    ///control topic pub
    nodes_poses_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    nodes_sensor_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    ///pixhawk data subs
    sub_pix_state = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &node_gen::cb_pix_state,this);
    
    ///px4 service client
    pathReset = nh.serviceClient<mavros_msgs::CommandBool>("/node_gen/path_reset");
    nodeRegister = nh.serviceClient<mavros_msgs::CommandBool>("/node_gen/register");
    pathReset = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    ///UAV control service server
    srv_mission_start = nh.advertiseService("/auto_mission", &node_gen::fn_mission_start, this);
}

node_gen::~node_gen(){}
