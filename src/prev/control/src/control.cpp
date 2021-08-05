#include "control.hpp"

bool exploration_control::register_home()
{
	std::cout<<"register_home"<<std::endl;
	mavros_msgs::CommandHome home_cmd;
	home_cmd.request.current_gps = true;
	bool success;
	success = set_home.call(home_cmd);

	if(success){
		std::cout<<"register success"<<std::endl;
		ros::Duration(2.1).sleep();
		// log
	    loglog ="home position registered";
	    log_pub_ros_info(LOG_IMPORTANT,loglog);
	}
	return success;
}
void exploration_control::offboard()
{
	mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if(set_mode.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        // log
        loglog ="Offboard gained";
        log_pub_ros_info(LOG_IMPORTANT,loglog);
    }
	std::cout<<"offboard"<<std::endl;
    offb_last_request = ros::Time::now();
}
void exploration_control::arm()
{
	std::cout<<"arm"<<std::endl;
    
	// log
	loglog = "Try arm";
	log_pub_ros_info(LOG_ERROR,loglog);

	mavros_msgs::CommandBool arm_cmd;

	arm_cmd.request.value = true;

	cmdArming.call(arm_cmd);
	arm_last_request = ros::Time::now();
}
void exploration_control::takeoff()
{    
	mavros_msgs::PositionTarget target;
	target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
	target.type_mask = position_control_type;
	target.position.x = 0;
	target.position.y = 0;
	target.position.z = takeoff_height;
	// target.yaw = Quat2Angle(pix_home.orientation).z;
	target.yaw = Quat2Angle(trans_home_quat).z;
	local_setpoint_raw_pub.publish(target);
	if(pix_state.mode != "OFFBOARD" && (ros::Time::now() - offb_last_request > ros::Duration(1.0))) offboard();
	else if(pix_state.armed == false && (ros::Time::now() - arm_last_request > ros::Duration(1.0))) arm();
	else if(pix_state.armed == true && 
			pix_state.mode == "OFFBOARD" &&
			pix_extstate.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR)
	{
		geometry_msgs::Pose dest;

		dest.position.x = 0;
		dest.position.y = 0;
		dest.position.z = takeoff_height;
		dest.orientation = trans_home_quat;
		// std::cout<<"arrival_check :" << arrival_check(pix_local, dest) << std::endl;
		if(!arrival_check(pix_local, dest)) local_setpoint_raw_pub.publish(target);
		else
		{
			// ROS_INFO("take off complete");
			// log
			loglog = "take off complete";
			log_pub_ros_info(LOG_IMPORTANT,loglog);
			takeoff_flag = true;
		}
	}
}

void exploration_control::hover()
{

	hover_flag = true;
	// log
	loglog = "hover";
	log_pub_ros_info(LOG_IMPORTANT,loglog);

	mavros_msgs::PositionTarget hover_target;

	if(hover_saved == false)
	{
		hover_saved = true;
		saved_point = pix_local;
	}

	hover_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	hover_target.type_mask = position_control_type;
    hover_target.position.x = saved_point.pose.pose.position.x;
    hover_target.position.y = saved_point.pose.pose.position.y;
    hover_target.position.z = saved_point.pose.pose.position.z;
    hover_target.yaw = Quat2Angle(saved_point.pose.pose.orientation).z;
	local_setpoint_raw_pub.publish(hover_target);
}

void exploration_control::hover_reset()
{
	hover_saved = false;
	hover_flag	= false;
}

void exploration_control::circle()
{
	// log
	loglog = "circle";
	log_pub_ros_info(LOG_IMPORTANT,loglog);
	mavros_msgs::PositionTarget circle_target;
	if (ros::Time::now() - circle_start < ros::Duration(2*M_PI*init_radius/max_vel)){
		circle_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		circle_target.type_mask = velocity_control_type;
	    circle_target.velocity.x = 0;
	    circle_target.velocity.y = -max_vel;
	    circle_target.velocity.z = 0.3 * (takeoff_height - pix_local.pose.pose.position.z);
	    circle_target.yaw_rate = max_vel/init_radius;
		local_setpoint_raw_pub.publish(circle_target);
	}
	else{
		circle_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
		circle_target.type_mask = position_control_type;
	    circle_target.position = pix_home.position;
	    circle_target.position.z = takeoff_height;
	    circle_target.yaw = Quat2Angle(pix_home.orientation).z;
		local_setpoint_raw_pub.publish(circle_target);
		geometry_msgs::Pose circle_test;
		circle_test.position = pix_home.position;
		circle_test.position.z = takeoff_height;
		circle_test.orientation = pix_home.orientation;
		if (arrival_check(pix_local, circle_test)){
			control_mode = wait_control;
			n_cpp::path_terminate term;
			path_termination.call(term);
		}
	}
}

void exploration_control::move()
{
	// log
	loglog = "move!";
	log_pub_ros_info(LOG_IMPORTANT,loglog);

	mavros_msgs::PositionTarget local_target;
	local_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	local_target.type_mask = velocity_control_type;
	double dist = distance(received_path.poses[waypoint_index].pose.position, pix_local.pose.pose.position);
	if (dist>0.5){
		local_target.velocity.x = max_vel*(received_path.poses[waypoint_index].pose.position.x - pix_local.pose.pose.position.x)/dist;
	    local_target.velocity.y = max_vel*(received_path.poses[waypoint_index].pose.position.y - pix_local.pose.pose.position.y)/dist;
	    local_target.velocity.z = max_vel*(received_path.poses[waypoint_index].pose.position.z - pix_local.pose.pose.position.z)/dist;
	    local_target.yaw_rate = 0.1* (Quat2Angle(received_path.poses[waypoint_index].pose.orientation).z - Quat2Angle(pix_local.pose.pose.orientation).z);
	}
	else{
		local_target.velocity.x = max_vel*(received_path.poses[waypoint_index].pose.position.x - pix_local.pose.pose.position.x);
	    local_target.velocity.y = max_vel*(received_path.poses[waypoint_index].pose.position.y - pix_local.pose.pose.position.y);
	    local_target.velocity.z = max_vel*(received_path.poses[waypoint_index].pose.position.z - pix_local.pose.pose.position.z);
	    local_target.yaw_rate = 0.1* (Quat2Angle(received_path.poses[waypoint_index].pose.orientation).z - Quat2Angle(pix_local.pose.pose.orientation).z);
	}
	local_setpoint_raw_pub.publish(local_target);
	if (arrival_check(pix_local, received_path.poses[waypoint_index].pose))
	{
		waypoint_index += 1;
	}
	if(waypoint_index == path_size){
		waypoint_index = 0;
		control_mode = wait_control;
	}
}

void exploration_control::spinOnce()
{
	switch(control_mode){
		case standby:
			mission_start_time = ros::Time::now();
			// log
		    loglog = "mission not started";
		    log_pub_ros_info(LOG_ERROR,loglog);
		    break;
		case wait_control:
			hover();
			break;
		case initiation:
			if (!home_registered){
				// log
			    loglog = "mission initiation";
			    log_pub_ros_info(LOG_ERROR,loglog);
				home_registered = register_home();
			} 
			else if (!takeoff_flag){
				takeoff();
				circle_start = ros::Time::now();
			}
			else {
				hover_reset();
				circle();
			}
			break;
		case local_control:
			move();
			break;
		case global_control:
			move();
			if(map_termination()){
				control_mode = control_end;
			}
			break;
		case control_end:
			hover();
			ros::Duration flight_time = ros::Time::now() - mission_start_time;
			std_msgs::Float64 pub_time;
			pub_time.data = flight_time.toSec();
			flight_time_pub.publish(pub_time);
			break;
	}
}