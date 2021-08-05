#include <ros/ros.h>
#include <time.h>

#include "mavros_msgs/PositionTarget.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/path.h"

#include "n_cpp/mission_terminate.h"
#include "n_cpp/next_step.h"
#include "n_cpp/collision_check.h"

#include <decomp_ros_utils/data_ros_utils.h>
#include <mpl_external_planner/ellipsoid_planner/ellipsoid_planner.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>

#include <sensor_msgs/point_cloud_conversion.h>

class N_control{
private:
	bool arrival;
	bool terminate;
	int index = 0;

    //sub
    ros::Subscriber sub_pix_state;
    ros::Subscriber sub_pix_extstate;
    ros::Subscriber sub_pix_poseLocal;
    ros::Subscriber sub_octomap_occu;
    ros::Subscriber sub_joy;

    // pub
    ros::Publisher local_setpoint_raw_pub;
    ros::Publisher pub_mission_time;

    //service client
    ros::ServiceClient mission_terminate;
    ros::ServiceClient next_step;

    // //service server
    // ros::ServiceServer srv_mission_start;
    void cb_pix_state    (const mavros_msgs::State::ConstPtr &msg);
    void cb_pix_extstate (const mavros_msgs::ExtendedState::ConstPtr &msg);
    void cb_pix_poseLocal(const nav_msgs::Odometry::ConstPtr &msg);
    void cb_joy			 (const sensor_msgs::Joy::ConstPtr &msg);

    ros::Timer ctrl_pub_timer;

	// Initialize planner
	double traj_dt, v_max, a_max, w, epsilon;
	double u_max_z, u_max;
	int max_num, num;
	bool use_3d;
	double robot_radius;
	Vec3f origin, dim;

public:
	ros::NodeHandle nh;
	void ctrl_timer(const ros::TimerEvent& event);
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

	bool arrived(nav_msgs::path path, int idx);
	bool next_local_collision(nav_msgs::path path, int idx); ////?????????????DUCKDUCKDUCK how to implement???????????????????????????????????????????????????????????????????????????????????????????????????????
	bool traj_follow(nav_msgs::path path, int idx);
	bool path_follow(nav_msgs::path path, int idx);

	void spinOnce();

    //service client
    ros::ServiceClient mission_terminate;
    ros::ServiceClient next_step;
    ros::ServiceClient collision_check;
    ros::ServiceClient cmd_arming;
    ros::ServiceClient set_mode;
    ros::ServiceClient set_home;

	N_control(ros::NodeHandle& n) : nh(n){

		nh.param("robot_r", robot_radius, 0.5);
		nh.param("origin_x", origin(0), 0.0);
		nh.param("origin_y", origin(1), 0.0);
		nh.param("origin_z", origin(2), 0.0);
		nh.param("range_x", dim(0), 0.0);
		nh.param("range_y", dim(1), 0.0);
		nh.param("range_z", dim(2), 0.0);
		nh.param("traj_dt", traj_dt, 0.03);
		nh.param("epsilon", epsilon, 1.0);
		nh.param("v_max", v_max, -1.0);
		nh.param("a_max", a_max, -1.0);
		nh.param("u_max", u_max, 1.0);
		nh.param("u_max_z", u_max_z, 1.0);
		nh.param("w", w, 10.);
		nh.param("num", num, 1);
		nh.param("max_num", max_num, -1);
		nh.param("use_3d", use_3d, false);
		
		///control topic pub
		local_setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        
		///mission time pub
		pub_mission_time = nh.advertise<std_msgs::Time>("/manuever_time", 30);

		///pixhawk data subs
		sub_pix_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &N_control::cb_pix_state,this);
        sub_pix_extstate = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10, &N_control::cb_pix_extstate,this);
        sub_pix_poseLocal = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/pose", 1, &N_control::cb_pix_local,this);
		sub_octomap = nh.subscribe<sensor_msgs::PointCloud>("/mavros/local_position/pose", 1, &N_control::cb_octomap,this);
		sub_joy = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &N_control::cb_joy,this);

        ///px4 service client
        mission_terminate = nh.serviceClient<mavros_msgs::CommandBool>("/n_path/mission_terminate");
	    next_step = nh.serviceClient<mavros_msgs::CommandBool>("/n_path/next_step");
	    collision_check = nh.serviceClient<mavros_msgs::CommandBool>("/n_octo/collision_check");
        cmd_arming = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        set_home = nh.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");

        ctrl_pub_timer = nh.createTimer(ros::Duration(traj_dt), &N_control::ctrl_timer, this);
	};
	~N_control(){};

}