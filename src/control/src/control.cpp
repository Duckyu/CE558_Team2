#include "control.hpp"

bool UAV_control::register_home()
{
	std::cout<<"register_home"<<std::endl;
	mavros_msgs::CommandHome home_cmd;
	home_cmd.request.current_gps = true;
	bool success;
	success = set_home.call(home_cmd);

	if(success){
		std::cout<<"register success"<<std::endl;
		home_altitude_amsl = pix_altitude.amsl;
		ros::Duration(2.1).sleep();
		// log
	    loglog ="home position registered";
	    log_pub_ros_info(LOG_IMPORTANT,loglog);
	}
	return success;
}
void UAV_control::offboard()
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
void UAV_control::arm()
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
void UAV_control::takeoff()
{    
	if(mission_local == false)
	{
		std::cout<<"takeoff global"<<std::endl;
    
		mavros_msgs::GlobalPositionTarget target;
		target.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
		target.type_mask = position_control_type;
		target.latitude = pix_home.geo.latitude;
		target.longitude = pix_home.geo.longitude;
		target.altitude = home_altitude_amsl + takeoff_height;
		target.yaw = Quat2Angle(pix_home.orientation).z;
		global_setpoint_raw_pub.publish(target);
		if(pix_state.mode != "OFFBOARD" && (ros::Time::now() - offb_last_request > ros::Duration(1.0))) offboard();
		else if(pix_state.armed == false && (ros::Time::now() - arm_last_request > ros::Duration(1.0))) arm();
		else if(pix_state.armed == true && 
				pix_state.mode == "OFFBOARD" &&
				pix_extstate.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR)
		{
			geometry_msgs::Pose dest;
			dest.position = pix_home.position;
			dest.position.z += takeoff_height;
			dest.orientation = pix_home.orientation;
			if(!arrival_check(pix_local, dest)) global_setpoint_raw_pub.publish(target);
			else takeoff_flag = true;
		}
	}
	else
	{
		std::cout<<"takeoff local"<<std::endl;
    
		mavros_msgs::PositionTarget target;
		target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		target.type_mask = position_control_type;
		target.position.x = 0;
		target.position.y = 0;
		target.position.z = takeoff_height;
		// target.yaw = Quat2Angle(pix_home.orientation).z;
		target.yaw = 0;
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
			dest.orientation.x = 0;
			dest.orientation.y = 0;
			dest.orientation.z = 0;
			dest.orientation.w = 1;
			std::cout<<"arrival_check :" << arrival_check(pix_local, dest) << std::endl;
			if(!arrival_check(pix_local, dest)) local_setpoint_raw_pub.publish(target);
			else
			{
				ROS_INFO("take off complete");
				takeoff_flag = true;
			}
		}
	}

	
}

void UAV_control::move2waypoint()
{
	if(mission_local == false)
	{
		mavros_msgs::GlobalPositionTarget target;
		target.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
		target.type_mask = position_control_type;
	    target.latitude = pix_home.geo.latitude;
	    target.longitude = pix_home.geo.longitude;
	    target.altitude = home_altitude_amsl + mission_height;
	    target.yaw = Quat2Angle(pix_home.orientation).z;
		global_setpoint_raw_pub.publish(target);

		geometry_msgs::Pose dest;
		dest.position = pix_home.position;
		dest.position.z += mission_height;
		dest.orientation = pix_home.orientation;
		if(arrival_check(pix_local, dest))
		{
			// recent_arrived_waypoint = pix_local;
			// waypoint_index += 1;
		}

		ros::Duration(0.05).sleep();
	}
	else
	{
		int waypoint_size = 2;
		mavros_msgs::PositionTarget target[waypoint_size];
		target[0].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
		target[0].type_mask = position_control_type;
	    target[0].position.x = 0;
		target[0].position.y = 0;
		target[0].position.z = mission_height;
	    target[0].yaw = 0;

		target[1].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
		target[1].type_mask = position_control_type;
	    target[1].position.x = 0;
		target[1].position.y = 0;
		target[1].position.z = mission_height;
	    target[1].yaw = 0;

		local_setpoint_raw_pub.publish(target[waypoint_index]);

		static int waypoint_index = 0;

		geometry_msgs::Pose dest[waypoint_size];
		dest[0].position.x = 0;
		dest[0].position.x = 0;
		dest[0].position.z = mission_height;
		dest[0].orientation.x = 0;
		dest[0].orientation.y = 0;
		dest[0].orientation.z = 0;
		dest[0].orientation.w = 1;
		
		dest[1].position.x = waypoint_x;
		dest[1].position.x = waypoint_y;
		dest[1].position.z = mission_height;
		dest[1].orientation.x = 0;
		dest[1].orientation.y = 0;
		dest[1].orientation.z = 0;
		dest[1].orientation.w = 1;

		if(arrival_check(pix_local, dest[waypoint_index]))
		{
			waypoint_index++;
			ROS_INFO("hihihi");
			if (waypoint_index >= waypoint_size)
			{
				arrival_flag = true;
				waypoint_index = 0;
			}
		}

		ros::Duration(0.05).sleep();
	}
	/*
	timeout_check();
	if(waypoint_index == 0)
	{
		mavros_msgs::GlobalPositionTarget target;
		target.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
		target.type_mask = position_control_type;
	    target.latitude = pix_home.geo.latitude;
	    target.longitude = pix_home.geo.longitude;
	    target.altitude = home_altitude_amsl + mission_height;
	    target.yaw = Quat2Angle(pix_home.orientation).z;
		global_setpoint_raw_pub.publish(target);

		geometry_msgs::Pose dest;
		dest.position = pix_home.position;
		dest.position.z += mission_height;
		dest.orientation = pix_home.orientation;
		if(arrival_check(pix_local, dest))
		{
			recent_arrived_waypoint = pix_local;
			waypoint_index += 1;
		}

		ros::Duration(0.05).sleep();
	}
	else if(waypoint_index == 1) 
	{
		startpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
		startpoint.type_mask = position_control_type;
	    startpoint.position.x = recent_arrived_waypoint.pose.pose.position.x + 1;
	    startpoint.position.y = recent_arrived_waypoint.pose.pose.position.y + 1;
	    startpoint.position.z = recent_arrived_waypoint.pose.pose.position.z;
	    startpoint.yaw = Quat2Angle(pix_home.orientation).z;
		local_setpoint_raw_pub.publish(startpoint);

		geometry_msgs::Pose dest;
		dest.position = startpoint.position;
		dest.orientation = pix_home.orientation;
		if(arrival_check(pix_local, dest))
		{
			recent_arrived_waypoint = pix_local;
			waypoint_index += 1;
		}

		ros::Duration(0.05).sleep();
	}
	else if(waypoint_index == 2) 
	{
		waypoint1.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
		waypoint1.type_mask = waypoint_control_type;
	    waypoint1.velocity.x = p_gain*(recent_arrived_waypoint.pose.pose.position.x + 10 - pix_local.pose.pose.position.x);
	    waypoint1.velocity.y = p_gain*(recent_arrived_waypoint.pose.pose.position.y - pix_local.pose.pose.position.y);
	    waypoint1.velocity.z = p_gain*(recent_arrived_waypoint.pose.pose.position.z - pix_local.pose.pose.position.z);
	    waypoint1.yaw = Quat2Angle(pix_home.orientation).z;
		local_setpoint_raw_pub.publish(waypoint1);

		geometry_msgs::Pose dest;
		dest.position = waypoint1.position;
		dest.orientation = pix_home.orientation;
		if(arrival_check(pix_local, dest))
		{
			recent_arrived_waypoint = pix_local;
			waypoint_index += 1;
		}

		ros::Duration(0.05).sleep();
	}
	else if(waypoint_index == 3) 
	{
		waypoint2.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
		waypoint2.type_mask = position_control_type;
	    waypoint2.position.x = recent_arrived_waypoint.pose.pose.position.x;
	    waypoint2.position.y = recent_arrived_waypoint.pose.pose.position.y;
	    waypoint2.position.z = recent_arrived_waypoint.pose.pose.position.z;
	    waypoint2.yaw = Quat2Angle(pix_home.orientation).z-M_PI;
		local_setpoint_raw_pub.publish(waypoint2);

		geometry_msgs::Pose dest;
		dest.position = waypoint2.position;
		dest.orientation = pix_home.orientation;
		if(arrival_check(pix_local, dest))
		{
			recent_arrived_waypoint = pix_local;
			waypoint_index += 1;
		}

		ros::Duration(0.05).sleep();
	}
	else if(waypoint_index == 3) 
	{
		waypoint3.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
		waypoint3.type_mask = waypoint_control_type;
	    waypoint3.velocity.x = p_gain*(recent_arrived_waypoint.pose.pose.position.x - 10 - pix_local.pose.pose.position.x);
	    waypoint3.velocity.y = p_gain*(recent_arrived_waypoint.pose.pose.position.y - pix_local.pose.pose.position.y);
	    waypoint3.velocity.z = p_gain*(recent_arrived_waypoint.pose.pose.position.z - pix_local.pose.pose.position.z);
	    waypoint3.yaw = Quat2Angle(pix_home.orientation).z-M_PI;
		local_setpoint_raw_pub.publish(waypoint3);

		geometry_msgs::Pose dest;
		dest.position = waypoint3.position;
		dest.orientation = pix_home.orientation;
		if(arrival_check(pix_local, dest))
		{
			recent_arrived_waypoint = pix_local;
			waypoint_index += 1;
		}

		ros::Duration(0.05).sleep();
	}
	else
	{
		ros::Duration(0.5).sleep();
		RTH();
	}
	*/
}
void UAV_control::hover()
{

	std::cout << !follower_start << hover_flag <<std::endl;
	timeout_check();
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

void UAV_control::hover_reset()
{
	hover_saved = false;
	human_lost_flag	= false;
}
void UAV_control::follow()
{
	std::cout << size(tracker_result.bounding_boxes) << std::endl;
	if (size(tracker_result.bounding_boxes) == 0) // detection condition??
	{
		// log
		loglog = "human not detected";
		log_pub_ros_info(LOG_IMMPORTANT,loglog);
		hover();
	}
	else
	{	
		static ros::Time last_detected = ros::Time::now();	
		int max_iter = 0;
		double max_prob = 0.0;
		for(int i=1;i<size(tracker_result.bounding_boxes);i++)
		{
			if(max_prob < tracker_result.bounding_boxes[i].probability)
			{
				//if(human_size(tracker_result.bounding_boxes[i].xmax - tracker_result.bounding_boxes[i].xmin, tracker_result.bounding_boxes[i].ymax - tracker_result.bounding_boxes[i].ymin))
				max_iter = i;
				max_prob = tracker_result.bounding_boxes[i].probability;
				//}
			}
		}
		// double tmp_pix_x = (960.0 - (box_in.bounding_boxes[0].xmin + box_in.bounding_boxes[0].xmax)/2.0)/960.0*7.6358/3.0;
		// double tmp_pix_y = -(540.0 - (box_in.bounding_boxes[0].ymin + box_in.bounding_boxes[0].ymax)/2.0)/540.0*4.2322/2.0;
		human_x = (tracker_result.bounding_boxes[max_iter].xmax + tracker_result.bounding_boxes[max_iter].xmin)/2.0;
		human_y = (tracker_result.bounding_boxes[max_iter].ymax + tracker_result.bounding_boxes[max_iter].ymin)/2.0;
		// LPF(center_x,center_y,center_x_old,center_y_old);

		aa;
		
		human_x_old = human_x;
		human_y_old = human_y;
	}
	

	// tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    // tf::Matrix3x3 m(q);
    // m.getRPY(curr_roll, curr_pitch, curr_yaw);
}
void UAV_control::RTH()
{
	ROS_INFO("RTH");
	static geometry_msgs::Pose planar_destination;
	geometry_msgs::Pose temp_destination;
	geometry_msgs::Quaternion current_quat;
	mavros_msgs::PositionTarget return_target;
	return_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

	if(mission_started == true)
	{
		mission_started = false;
		temp_destination.position = pix_home.position;
		temp_destination.position.z = pix_local.pose.pose.position.z;
		temp_destination.orientation = pix_local.pose.pose.orientation;
		planar_destination = temp_destination;
	}
	
	while(!arrival_check(pix_local, planar_destination))
	{
		timeout_check();
		return_target.type_mask = waypoint_control_type;
	    return_target.velocity.x = p_gain*(pix_home.position.x- pix_local.pose.pose.position.x);
	    return_target.velocity.y = p_gain*(pix_home.position.y - pix_local.pose.pose.position.y);
	    return_target.velocity.z = p_gain*(mission_height - pix_local.pose.pose.position.z);
	    return_target.yaw = Quat2Angle(current_quat).z;
		local_setpoint_raw_pub.publish(return_target);
		ros::Duration(0.05).sleep();
	}

	planar_destination.orientation = pix_home.orientation;
	while(!arrival_check(pix_local, planar_destination))
	{
		timeout_check();
		return_target.type_mask = position_control_type;
	    return_target.position.x = pix_home.position.x;
	    return_target.position.y = pix_home.position.y;
	    return_target.position.z = mission_height;
	    return_target.yaw = Quat2Angle(pix_home.orientation).z;
		local_setpoint_raw_pub.publish(return_target);
		ros::Duration(0.05).sleep();
	}
	ros::Duration(0.5).sleep();

	planar_destination.position.z = mission_height - 10;
	while(!arrival_check(pix_local, planar_destination))
	{
		timeout_check();
		return_target.type_mask = position_control_type;
	    return_target.position.x = pix_home.position.x;
	    return_target.position.y = pix_home.position.y;
	    return_target.position.z = mission_height - 10;
	    return_target.yaw = Quat2Angle(pix_home.orientation).z;
		local_setpoint_raw_pub.publish(return_target);
		ros::Duration(0.05).sleep();
	}
	ros::Duration(0.5).sleep();

	planar_destination.position.z = mission_height - 20;
	while(!arrival_check(pix_local, planar_destination))
	{
		timeout_check();
		return_target.type_mask = position_control_type;
	    return_target.position.x = pix_home.position.x;
	    return_target.position.y = pix_home.position.y;
	    return_target.position.z = mission_height - 20;
	    return_target.yaw = Quat2Angle(pix_home.orientation).z;
		local_setpoint_raw_pub.publish(return_target);
		ros::Duration(0.05).sleep();
	}
	ros::Duration(0.5).sleep();

	planar_destination.position.z = takeoff_height;
	while(!arrival_check(pix_local, planar_destination))
	{
		timeout_check();
		return_target.type_mask = position_control_type;
	    return_target.position.x = pix_home.position.x;
	    return_target.position.y = pix_home.position.y;
	    return_target.position.z = takeoff_height;
	    return_target.yaw = Quat2Angle(pix_home.orientation).z;
		local_setpoint_raw_pub.publish(return_target);
		ros::Duration(0.05).sleep();
	}
	ros::Duration(0.5).sleep();

	while(pix_extstate.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
	{
		double landing_old;
    	double landing_cur;
    	ros::Time landing_time;

		timeout_check();
		return_target.type_mask = velocity_control_type;
        return_target.velocity.z = (-1)*landing_vel;
        if(ros::Time::now() - landing_time > ros::Duration(4.0)){
            landing_old = landing_cur; // Change old height value
            landing_cur = pix_local.pose.pose.position.z; // Change current height value
            landing_time = ros::Time::now();

            // landing clear condition
            // old height value > cur height value and the difference is less than 50cm
            if((landing_old > landing_cur) && abs(landing_old - landing_cur) < 0.5){
                return_target.type_mask = position_control_type;
                return_target.velocity.z = 0;
                return_target.position.x = pix_local.pose.pose.position.x;
                return_target.position.y = pix_local.pose.pose.position.y;
                return_target.position.z = 0;

                mavros_msgs::CommandBool cmd;
                cmd.request.value = false;
                cmdArming.call(cmd);

                if(pix_state.armed){
                    // log
                    loglog = "Landing on going";
                    log_pub_ros_info(LOG_ERROR,loglog);
                }
            }
        }
        local_setpoint_raw_pub.publish(return_target);
	}
	if(pix_extstate.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
	{
		// log
        loglog = "Landing clear";
        log_pub_ros_info(LOG_IMPORTANT,loglog);
	}
}

void UAV_control::spinOnce()
{
	if(mission_started == true)
	{
		timeout_check();
		if(!timeout_flag){
			if (!home_registered) home_registered = register_home();
			else if (!takeoff_flag) takeoff();
			else if (!arrival_flag) move2waypoint();
			else if (!follower_start) hover();
			else follow();
		}
		else RTH();
	}
	else
	{
		mission_start_time = ros::Time::now();
		std::cout<<"mission not started"<<std::endl;
	}
}