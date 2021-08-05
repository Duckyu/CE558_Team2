#define standby 0
#define wait_control 1
#define initiation 2
#define local_control 3
#define global_control 4
#define control_end 5

#define LOG_ERROR 0
#define LOG_NORMAL 1
#define LOG_EMERGENCY 2
#define LOG_IMPORTANT 3

# define M_PI 3.14159265358979323846

#include <ros/ros.h>
#include <time.h>
#include <cmath>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

//Pix message
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/HomePosition.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"

// custom UAV Mission
#include "n_cpp/mission_start.h"
#include "n_cpp/init_step.h"
#include "n_cpp/local_path.h"
#include "n_cpp/global_path.h"
#include "n_cpp/end.h"
#include "n_cpp/path_terminate.h"


#include <n_cpp/log_data.h>

class exploration_control
{
    private:

    //sub
    ros::Subscriber sub_pix_state;
    ros::Subscriber sub_pix_extstate;
    ros::Subscriber sub_pix_poseLocal;
    ros::Subscriber sub_pix_home;


    // pub
    ros::Publisher local_setpoint_raw_pub;
    ros::Publisher pub_log_data;
    ros::Publisher flight_time_pub;

    //service client
    ros::ServiceClient cmdArming;
    ros::ServiceClient set_mode;
    ros::ServiceClient set_home;
    ros::ServiceClient path_termination;

    //service server
    ros::ServiceServer srv_mission_start;
    ros::ServiceServer srv_init_step;
    ros::ServiceServer srv_local_path;
    ros::ServiceServer srv_global_path;
    ros::ServiceServer srv_end;

    /// extras.cpp
    geometry_msgs::Vector3 Quat2Angle(geometry_msgs::Quaternion quat);
    double distance(geometry_msgs::Point A, geometry_msgs::Point B);
    bool arrival_check(nav_msgs::Odometry UAV_position, geometry_msgs::Pose destination);
    bool map_termination();
    void log_pub_ros_info(int color, std::string log);
    double LPF_filter(double prev_value, double curr_value, double LPF_gain);
    double vel_saturation(double vel);
    ros::Time initial_arrival;
    std::string loglog_old;
    std::string loglog;


    ///callback function
    void cb_pix_state(const mavros_msgs::State::ConstPtr& msg);
    void cb_pix_extstate(const mavros_msgs::ExtendedState::ConstPtr& msg);
    void cb_pix_local(const nav_msgs::Odometry::ConstPtr &msg);
    void cb_pix_home(const mavros_msgs::HomePosition::ConstPtr &msg);
    bool fn_mission_start(n_cpp::mission_start::Request &request,
                          n_cpp::mission_start::Response &response);
    bool fn_init_step(n_cpp::init_step::Request &request,
                        n_cpp::init_step::Response &response);
    bool fn_local_path(n_cpp::local_path::Request &request,
                           n_cpp::local_path::Response &response);
    bool fn_global_path(n_cpp::global_path::Request &request,
                           n_cpp::global_path::Response &response);
    bool fn_end(n_cpp::end::Request &request,
                           n_cpp::end::Response &response);


    //control capsule function
    bool register_home();
    void offboard();
    void arm();
    void takeoff();
    void move();
    void hover();
    void hover_reset();
    void circle();

    ///position target control type arranged
    int velocity_control_type = mavros_msgs::PositionTarget::IGNORE_AFX|
                                       mavros_msgs::PositionTarget::IGNORE_AFY|
                                       mavros_msgs::PositionTarget::IGNORE_AFZ|
                                       mavros_msgs::PositionTarget::IGNORE_PX|
                                       mavros_msgs::PositionTarget::IGNORE_PY|
                                       mavros_msgs::PositionTarget::IGNORE_PZ|
                                       mavros_msgs::PositionTarget::IGNORE_YAW;

    int position_control_type = mavros_msgs::PositionTarget::IGNORE_AFX|
                                       mavros_msgs::PositionTarget::IGNORE_AFY|
                                       mavros_msgs::PositionTarget::IGNORE_AFZ|
                                       mavros_msgs::PositionTarget::IGNORE_VX|
                                       mavros_msgs::PositionTarget::IGNORE_VY|
                                       mavros_msgs::PositionTarget::IGNORE_VZ|
                                       mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    int yaw_align_control_type = mavros_msgs::PositionTarget::IGNORE_AFX|
                                        mavros_msgs::PositionTarget::IGNORE_AFY|
                                        mavros_msgs::PositionTarget::IGNORE_AFZ|
                                        mavros_msgs::PositionTarget::IGNORE_VX|
                                        mavros_msgs::PositionTarget::IGNORE_VY|
                                        mavros_msgs::PositionTarget::IGNORE_VZ|
                                        mavros_msgs::PositionTarget::IGNORE_YAW;

    int waypoint_control_type = mavros_msgs::PositionTarget::IGNORE_AFX|
                                       mavros_msgs::PositionTarget::IGNORE_AFY|
                                       mavros_msgs::PositionTarget::IGNORE_AFZ|
                                       mavros_msgs::PositionTarget::IGNORE_PX|
                                       mavros_msgs::PositionTarget::IGNORE_PY|
                                       mavros_msgs::PositionTarget::IGNORE_PZ|
                                       mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    public:
    ros::NodeHandle nh;

    /// parameter in launch file
    double takeoff_height;
    double arrival_dist_thres;
    double arrival_yaw_thres;
    double max_vel;
    double landing_vel;
    double p_gain;
    double i_gain;
    double d_gain;

    double img_x;
    double img_y;
    double safe_distance;
    double unit_len_height;
    double unit_len_planar;


    /// pix information
    mavros_msgs::State pix_state; /// armed, guided, manual_input, mode, system_status
    mavros_msgs::ExtendedState pix_extstate;
    nav_msgs::Odometry pix_local; /// amsl, local
    mavros_msgs::HomePosition pix_home;
    geometry_msgs::Quaternion trans_home_quat;

    // waypoint information
    // #define standby 0
    // #define wait 1
    // #define init 2
    // #define local_path 3
    // #define global_path 4
    // #define end 5
    int control_mode = standby;


    /// common control pub msg
    // mavros_msgs::GlobalPositionTarget;
    // mavros_msgs::PositionTarget;

    /// fn_mission_start();
    bool mission_started = false;

    /// fn_init_step();
    double init_radius = 5.0;

    /// register_home()
    bool home_registered = false;
    // geometry_msgs::Quaternion home_quat;

    int path_size;
    nav_msgs::Path received_path;

    /// offboard()
    ros::Time offb_last_request;

    /// arm();
    ros::Time arm_last_request;

    /// takeoff();
    bool takeoff_flag = false;

    /// circle()
    ros::Time circle_start;
    int circle_time = 0;
    
    /// move2waypoint();
    int waypoint_index = 0;
    mavros_msgs::PositionTarget startpoint;
    bool arrival_flag = false;


    /// for test flight
    nav_msgs::Odometry recent_arrived_waypoint;
    
    /// hover();
    bool hover_saved = false;
    bool hover_flag = false;
    nav_msgs::Odometry saved_point;

    ros::Time mission_start_time;

    /// For main function
    void spinOnce();

    exploration_control(){
        ///control topic pub
        local_setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        ///log topic pub
        pub_log_data = nh.advertise<n_cpp::log_data>("/exploration_control/log_data", 30);

        ///mission time pub
        flight_time_pub = nh.advertise<std_msgs::Float64>("/exploration_control/flight_time", 30);

        ///pixhawk data subs
        sub_pix_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &exploration_control::cb_pix_state,this);
        sub_pix_extstate = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10, &exploration_control::cb_pix_extstate,this);
        sub_pix_poseLocal = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &exploration_control::cb_pix_local,this);
        sub_pix_home = nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 1, &exploration_control::cb_pix_home,this);
        
        ///px4 service client
        cmdArming = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        set_home = nh.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
        path_termination = nh.serviceClient<n_cpp::path_terminate>("/exploration_control/path_terminate");

        ///UAV control service server
        srv_mission_start = nh.advertiseService("/exploration_control/mission_start", &exploration_control::fn_mission_start, this);
        srv_init_step = nh.advertiseService("/exploration_control/initiation", &exploration_control::fn_init_step, this);
        srv_local_path = nh.advertiseService("/exploration_control/local_path", &exploration_control::fn_local_path, this);
        srv_global_path = nh.advertiseService("/exploration_control/global_path", &exploration_control::fn_global_path, this);
        srv_end = nh.advertiseService("/exploration_control/end", &exploration_control::fn_end, this);
    }
    ~exploration_control(){}
};
