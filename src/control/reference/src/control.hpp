#define wait_mode 0
#define takeoff_mode 1
#define waypoint_mode 2
#define hovering_mode 3
#define following_mode 4
#define RTH_mode 5

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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/Altitude.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <darknet_ros_msgs/BoundingBoxes.h>

// custom UAV Mission
// #include "drone_contest/mission_start.h"
// #include "drone_contest/return_home.h"
// #include "drone_contest/follower_start.h"
#include "jane_ui/automatic_mission_start.h"
#include "jane_ui/return_home.h"
#include "jane_ui/next_mission.h"
#include <drone_contest/log_data.h>

//darknet_ros_msgs::BoundingBoxes box_in;
//bool box_in_check=false;
//bool detection_tracking=false;

class UAV_control
{
    private:

    //sub
    ros::Subscriber sub_pix_state;
    ros::Subscriber sub_pix_extstate;
    ros::Subscriber sub_pix_poseLocal;
    // ros::Subscriber sub_pix_GPSLocal;
    ros::Subscriber sub_pix_altitude;
    ros::Subscriber sub_pix_home;
    ros::Subscriber sub_tracker_result;
    ros::Subscriber detect_sub;


    // pub
    ros::Publisher local_setpoint_raw_pub;
    ros::Publisher global_setpoint_raw_pub;
    ros::Publisher pub_log_data;
    ros::Publisher pub_mission_time;

    //service client
    ros::ServiceClient cmdArming;
    ros::ServiceClient set_mode;
    ros::ServiceClient set_home;

    //service server
    ros::ServiceServer srv_mission_start;
    ros::ServiceServer srv_return_home;
    ros::ServiceServer srv_follower_start;

    /// extras.cpp
    geometry_msgs::Vector3 Quat2Angle(geometry_msgs::Quaternion quat);
    double distance(geometry_msgs::Point A, geometry_msgs::Point B);
    bool arrival_check(nav_msgs::Odometry UAV_position, geometry_msgs::Pose destination);
    bool arrival_check_wo_yaw(nav_msgs::Odometry UAV_position, geometry_msgs::Pose destination);
    void log_pub_ros_info(int color, std::string log);
    void timeout_check();
    double LPF_filter(double prev_value, double curr_value, double LPF_gain);
    double vel_saturation(double vel);
    ros::Time initial_arrival;
    std::string loglog_old;
    std::string loglog;


    ///callback function
    void cb_pix_state(const mavros_msgs::State::ConstPtr& msg);
    void cb_pix_extstate(const mavros_msgs::ExtendedState::ConstPtr& msg);
    void cb_pix_local(const nav_msgs::Odometry::ConstPtr &msg);
    // void cb_pix_GPSlocal(const nav_msgs::Odometry::ConstPtr &msg);
    void cb_pix_altitude(const mavros_msgs::Altitude::ConstPtr &msg);
    void cb_pix_home(const mavros_msgs::HomePosition::ConstPtr &msg);
    void cb_tracker_result(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
    // void cb_detection_tracking(const std_msgs::Bool::ConstPtr &msg);
    // bool fn_mission_start(drone_contest::mission_start::Request &request,
    //                       drone_contest::mission_start::Response &response);
    // bool fn_return_home(drone_contest::return_home::Request &request,
    //                     drone_contest::return_home::Response &response);
    // bool fn_follower_start(drone_contest::follower_start::Request &request,
    //                       drone_contest::follower_start::Response &response);
    bool fn_mission_start(jane_ui::automatic_mission_start::Request &request,
                          jane_ui::automatic_mission_start::Response &response);
    bool fn_return_home(jane_ui::return_home::Request &request,
                        jane_ui::return_home::Response &response);
    bool fn_follower_start(jane_ui::next_mission::Request &request,
                          jane_ui::next_mission::Response &response);        


    //control capsule function
    bool register_home();
    void offboard();
    void arm();
    void takeoff();
    void move2waypoint();
    void hover();
    void hover_reset();
    void follow();
    void RTH();


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
    std::string tracker_topic = "/darknet_ros/bounding_boxes";
    double geofence_longitude;
    double geofence_latitude;
    double takeoff_height;
    double mission_height;
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
    bool mission_local = false;
    double unit_len_height;
    double unit_len_planar;
    double waypoint_x;
    double waypoint_y;
    double waypoint_latitude;
    double waypoint_longitude;
    double timeout; // second
    int mode;


    /// pix information
    mavros_msgs::State pix_state; /// armed, guided, manual_input, mode, system_status
    mavros_msgs::ExtendedState pix_extstate;
    nav_msgs::Odometry pix_local; /// amsl, local
    // geometry_msgs::nav_msgs::Odometry pix_GPSlocal;
    mavros_msgs::Altitude pix_altitude;
    mavros_msgs::HomePosition pix_home;
    geometry_msgs::Quaternion trans_home_quat;

    /// follower input
    darknet_ros_msgs::BoundingBoxes tracker_result;
    bool detection_tracking = false;

    // waypoint information


    /// common control pub msg
    // mavros_msgs::GlobalPositionTarget;
    // mavros_msgs::PositionTarget;

    /// fn_mission_start();
    bool mission_started = false;

    /// fn_return_home();
    bool return_home = false;

    /// fn_follower_start();
    bool follower_start = false;

    /// register_home()
    bool home_registered = false;
    double home_altitude_amsl;
    // geometry_msgs::Quaternion home_quat;

    /// offboard()
    ros::Time offb_last_request;

    /// arm();
    ros::Time arm_last_request;

    /// takeoff();
    bool takeoff_flag = false;
    
    /// move2waypoint();
    int waypoint_index = 0;
    mavros_msgs::PositionTarget startpoint;
    bool arrival_flag = false;


    /// for test flight
    nav_msgs::Odometry recent_arrived_waypoint;
    mavros_msgs::PositionTarget waypoint1;
    mavros_msgs::PositionTarget waypoint2;
    mavros_msgs::PositionTarget waypoint3;
    
    /// hover();
    bool hover_saved = false;
    nav_msgs::Odometry saved_point;
    bool human_lost_flag = false;

    /// follow();
    ros::Time last_detected;
    // double human_x;
    // double human_y;
    // double human_x_old;
    // double human_y_old;
    
    /// RTH();
    int RTH_index = 0;
    nav_msgs::Odometry return_point;
    ros::Time mission_start_time;

    /// timeout_check();
    bool timeout_flag = false;

    /// For main function
    void spinOnce();

    UAV_control(){
        ///control topic pub
        local_setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        global_setpoint_raw_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 10);

        ///log topic pub
        pub_log_data = nh.advertise<drone_contest::log_data>("/jane/log_data", 30);

        ///mission time pub
        pub_mission_time = nh.advertise<std_msgs::Time>("/drone_contest/flight_time", 30);

        ///pixhawk data subs
        sub_pix_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &UAV_control::cb_pix_state,this);
        sub_pix_extstate = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10, &UAV_control::cb_pix_extstate,this);
        sub_pix_poseLocal = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &UAV_control::cb_pix_local,this);
        // sub_pix_GPSLocal = nh.subscribe<geometry_msgs::nav_msgs::Odometry>("/mavros/global_position/local", 1, &UAV_control::cb_pix_GPSlocal,this);
        sub_pix_altitude = nh.subscribe<mavros_msgs::Altitude>("/mavros/altitude", 1, &UAV_control::cb_pix_altitude,this);
        sub_pix_home = nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 1, &UAV_control::cb_pix_home,this);
        
        ///tracker data subs
        sub_tracker_result = nh.subscribe<darknet_ros_msgs::BoundingBoxes>(tracker_topic, 1, &UAV_control::cb_tracker_result,this);
        // detect_sub = nh.subscribe<std_msgs::Bool>("/detection_tracking", 10, &UAV_control::cb_detection_tracking,this);

        ///px4 service client
        cmdArming = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        set_home = nh.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");

        ///UAV control service server
        // srv_mission_start = nh.advertiseService("/drone_contest/mission_start", &UAV_control::fn_mission_start, this);
        // srv_return_home = nh.advertiseService("/drone_contest/return_home", &UAV_control::fn_return_home, this);
        // srv_follower_start = nh.advertiseService("/drone_contest/follower_start", &UAV_control::fn_follower_start, this);
        srv_mission_start = nh.advertiseService("/auto_mission", &UAV_control::fn_mission_start, this);
        srv_return_home = nh.advertiseService("/return_home", &UAV_control::fn_return_home, this);
        srv_follower_start = nh.advertiseService("/next_mission", &UAV_control::fn_follower_start, this);
    }
    ~UAV_control(){}
};
