#include "control.hpp"

geometry_msgs::Vector3 UAV_control::Quat2Angle(geometry_msgs::Quaternion quat)
{
    geometry_msgs::Vector3 res;
    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(res.x, res.y, res.z);
    return res;
}

double UAV_control::distance(geometry_msgs::Point A, geometry_msgs::Point B)
{
	return pow(pow(A.x - B.x, 2) + pow(A.y - B.y, 2) + pow(A.z - B.z, 2), 0.5);
}

bool UAV_control::arrival_check(nav_msgs::Odometry UAV_position, geometry_msgs::Pose destination)
{
	bool check_result = false;
	geometry_msgs::Pose UAV_pose = UAV_position.pose.pose;
	// std::cout << "Quat2Angle(UAV_pose.orientation).z - Quat2Angle(destination.orientation).z" << Quat2Angle(UAV_pose.orientation).z - Quat2Angle(destination.orientation).z << std::endl;
	if(distance(UAV_pose.position, destination.position) < arrival_dist_thres &&
	   Quat2Angle(UAV_pose.orientation).z - Quat2Angle(destination.orientation).z < arrival_yaw_thres)
	{
		if(ros::Time::now() - initial_arrival > ros::Duration(1.0))
		{
			check_result = true;
		}
	}
	else initial_arrival = ros::Time::now();

	return check_result;
}

bool UAV_control::arrival_check_wo_yaw(nav_msgs::Odometry UAV_position, geometry_msgs::Pose destination)
{
	bool check_result = false;
	geometry_msgs::Pose UAV_pose = UAV_position.pose.pose;
	// std::cout << "distance :" << distance(UAV_pose.position, destination.position) << std::endl;
	if(distance(UAV_pose.position, destination.position) < arrival_dist_thres)
	{
		// if(ros::Time::now() - initial_arrival > ros::Duration(0.02))
		// {
		check_result = true;
		// }
	}
	else initial_arrival = ros::Time::now();

	return check_result;
}

void UAV_control::log_pub_ros_info(int color, std::string log)
{
	drone_contest::log_data pix2uav_log_data;
    pix2uav_log_data.string_color = color;
    pix2uav_log_data.log_string = log;

    if(log != loglog_old){
      loglog_old = log;
      pub_log_data.publish(pix2uav_log_data);
    }
}

void UAV_control::timeout_check()
{
	if(ros::Time::now() - mission_start_time > ros::Duration(timeout))
	{
		timeout_flag = true;
	}
}

double UAV_control::LPF_filter(double prev_value, double curr_value, double LPF_gain){
    return (1.0-LPF_gain)*(prev_value) + LPF_gain*curr_value;
}

double UAV_control::vel_saturation(double vel){
    double sat = 1.5;
    if (vel>sat)
        vel = sat;
    if (vel<-sat)
        vel = -sat;
    return vel;
}
