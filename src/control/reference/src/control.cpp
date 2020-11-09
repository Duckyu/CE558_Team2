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
		target.yaw = Quat2Angle(trans_home_quat).z;
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
			dest.orientation = trans_home_quat;
			if(!arrival_check(pix_local, dest)) global_setpoint_raw_pub.publish(target);
			else takeoff_flag = true;
		}
	}
	else
	{
		// std::cout<<"takeoff local"<<std::endl;
    
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
		// log
		loglog = "move to waypoint ";
		log_pub_ros_info(LOG_IMPORTANT,loglog);

		geometry_msgs::Point wp;
		wp.x = waypoint_x;
		wp.y = waypoint_y;
		wp.z = mission_height;
		geometry_msgs::Point start_point;
		start_point = pix_home.position;
		start_point.z = mission_height;
		double wp_dist = distance(start_point, wp);
		// std::cout << "wp_dist :" << wp_dist << std::endl;
		int height_step = (mission_height - takeoff_height) / unit_len_height;
		int planar_step = wp_dist / unit_len_planar;
		int waypoint_size = height_step + planar_step;

		mavros_msgs::PositionTarget target[waypoint_size];
		geometry_msgs::Pose dest[waypoint_size];


		for(int height_index = 0; height_index < height_step; ++height_index)
		{
			target[height_index].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			target[height_index].type_mask = position_control_type;
		    target[height_index].position.x = 0;
			target[height_index].position.y = 0;
			target[height_index].position.z = takeoff_height + (mission_height - takeoff_height)*(height_index+1.0)/(height_step*1.0);
		    target[height_index].yaw = Quat2Angle(trans_home_quat).z;
			dest[height_index].position = target[height_index].position;
			dest[height_index].orientation = trans_home_quat;
		}

		for(int planar_index = height_step; planar_index < waypoint_size; ++planar_index)
		{
			target[planar_index].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			target[planar_index].type_mask = position_control_type;
		    target[planar_index].position.x = waypoint_x*(planar_index - height_step + 1.0)/(planar_step*1.0);
			target[planar_index].position.y = waypoint_y*(planar_index - height_step + 1.0)/(planar_step*1.0);
			target[planar_index].position.z = mission_height;
		    target[planar_index].yaw = Quat2Angle(trans_home_quat).z;
			dest[planar_index].position = target[planar_index].position;
			dest[planar_index].orientation = trans_home_quat;
		}

		if(mode == 0)
		{
			local_setpoint_raw_pub.publish(target[waypoint_index]);
		}
		else if(mode == 1)
		{
			mavros_msgs::PositionTarget vel_target;
			vel_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			vel_target.type_mask = velocity_control_type;
			vel_target.velocity.x = vel_saturation(max_vel * (dest[waypoint_index].position.x - pix_local.pose.pose.position.x));
			vel_target.velocity.y = vel_saturation(max_vel * (dest[waypoint_index].position.y - pix_local.pose.pose.position.y));
			vel_target.velocity.z = vel_saturation(max_vel * (dest[waypoint_index].position.z - pix_local.pose.pose.position.z));
			vel_target.yaw_rate = 0.1* (Quat2Angle(dest[waypoint_index].orientation).z - Quat2Angle(pix_local.pose.pose.orientation).z);
			local_setpoint_raw_pub.publish(vel_target);
		}


		if(arrival_check(pix_local, dest[waypoint_index]))
		{
			// log
			loglog = "waypoint " + std::to_string(waypoint_index) + " arrived among " + std::to_string(waypoint_size) + " number of waypoints";
			log_pub_ros_info(LOG_IMPORTANT,loglog);

			waypoint_index++;
			
			if (waypoint_index >= waypoint_size)
			{
				std::cout << "arrived" <<std::endl;
				arrival_flag = true;
				waypoint_index = 0;
			}
			// else
			// {
			// 	local_setpoint_raw_pub.publish(target[waypoint_index]);
			// }
		}
		ros::Duration(0.05).sleep();
	}
}
void UAV_control::hover()
{
	// log
	loglog = "wating for follower";
	log_pub_ros_info(LOG_IMPORTANT,loglog);

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
	// ros::Time last_detected = ros::Time::now();	
	int max_iter = 0;
	double max_prob = 0.0;
	for(int i = 1;i < tracker_result.bounding_boxes.size();++i)
	{
		if(max_prob < tracker_result.bounding_boxes[i].probability)
		{
			//if(human_size(tracker_result.bounding_boxes[i].xmax - tracker_result.bounding_boxes[i].xmin, tracker_result.bounding_boxes[i].ymax - tracker_result.bounding_boxes[i].ymin))
			max_iter = i;
			max_prob = tracker_result.bounding_boxes[i].probability;
			//}
		}
	}

	mavros_msgs::PositionTarget vel_input;
	mavros_msgs::PositionTarget prev_vel_input;
	vel_input.coordinate_frame = 1;
	vel_input.type_mask = velocity_control_type;
	if (ros::Time::now() - last_detected > ros::Duration(0.2))
	{
		hover_reset();
		hover();
	}
	else if(tracker_result.bounding_boxes.size() != 0)
	{

	    double tmp_pix_x = (960.0 - (tracker_result.bounding_boxes[max_iter].xmin + tracker_result.bounding_boxes[max_iter].xmax)/2.0)/960.0*7.6358/2.5;
	    double tmp_pix_y = -(540.0 - (tracker_result.bounding_boxes[max_iter].ymin + tracker_result.bounding_boxes[max_iter].ymax)/2.0)/540.0*4.2322/1.5;

	    double tmp_vx_in = tmp_pix_x*cos(Quat2Angle(pix_local.pose.pose.orientation).z) - tmp_pix_y*sin(Quat2Angle(pix_local.pose.pose.orientation).z);
	    double tmp_vy_in = tmp_pix_y*cos(Quat2Angle(pix_local.pose.pose.orientation).z) + tmp_pix_x*sin(Quat2Angle(pix_local.pose.pose.orientation).z);

	    vel_input.velocity.x = vel_saturation(LPF_filter(prev_vel_input.velocity.x, tmp_vx_in, 0.5));
	    vel_input.velocity.y = vel_saturation(LPF_filter(prev_vel_input.velocity.y, tmp_vy_in, 0.5));
	    vel_input.velocity.z = (mission_height - pix_local.pose.pose.position.z)*0.3;
	    //vel_input.twist.angular.z = atan2(goal_y-curr_y,goal_x-curr_x)*0.3;

	    local_setpoint_raw_pub.publish(vel_input);
	    ros::Duration(0.05).sleep();
	    // std::cout << tracker_result.bounding_boxes[max_iter].xmin << " " << vel_input.velocity.x << " " << vel_input.velocity.y << std::endl;
	    prev_vel_input = vel_input;
	    // box_in_check = false;
	}
	else
	{
		hover_reset();
		hover();
	}

	// human_x = (tracker_result.bounding_boxes[max_iter].xmax + tracker_result.bounding_boxes[max_iter].xmin)/2.0;
	// human_y = (tracker_result.bounding_boxes[max_iter].ymax + tracker_result.bounding_boxes[max_iter].ymin)/2.0;
	// // LPF(center_x,center_y,center_x_old,center_y_old);

	
	// human_x_old = human_x;
	// human_y_old = human_y;
}
void UAV_control::RTH()
{
	ROS_INFO("RTH");

	// log
	loglog = "move to Home waypoint";
	log_pub_ros_info(LOG_IMPORTANT,loglog);

	geometry_msgs::Pose RTH_destination;
	geometry_msgs::Pose temp_destination;
	geometry_msgs::Quaternion current_quat;
	mavros_msgs::PositionTarget return_target;
	return_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	/////////////////////////////////////////////////////////////////
	int height_step = (mission_height - takeoff_height) / unit_len_height;

	mavros_msgs::PositionTarget RTHtarget[height_step +1];
	geometry_msgs::Pose dest[height_step +1];

	RTHtarget[0].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	RTHtarget[0].type_mask = position_control_type;
    RTHtarget[0].position.x = 0;
	RTHtarget[0].position.y = 0;
	RTHtarget[0].position.z = mission_height;
    RTHtarget[0].yaw = Quat2Angle(trans_home_quat).z;
	dest[0].position = RTHtarget[0].position;
	dest[0].orientation = trans_home_quat;

	for(int height_index = 1; height_index < height_step + 1; ++height_index)
	{
		RTHtarget[height_index].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
		RTHtarget[height_index].type_mask = position_control_type;
	    RTHtarget[height_index].position.x = 0;
		RTHtarget[height_index].position.y = 0;
		RTHtarget[height_index].position.z = mission_height - (mission_height - takeoff_height) * (height_index + 1.0) / (height_step*1.0);
	    RTHtarget[height_index].yaw = Quat2Angle(trans_home_quat).z;
		dest[height_index].position = RTHtarget[height_index].position;
		dest[height_index].orientation = trans_home_quat;
	}

	if(mode == 0)
	{
		local_setpoint_raw_pub.publish(RTHtarget[RTH_index]);
	}
	else if(mode == 1)
	{
		mavros_msgs::PositionTarget vel_target;
		vel_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
		vel_target.type_mask = velocity_control_type;
		vel_target.velocity.x = vel_saturation(max_vel * (dest[RTH_index].position.x - pix_local.pose.pose.position.x));
		vel_target.velocity.y = vel_saturation(max_vel * (dest[RTH_index].position.y - pix_local.pose.pose.position.y));
		vel_target.velocity.z = vel_saturation(max_vel * (dest[RTH_index].position.z - pix_local.pose.pose.position.z));
		vel_target.yaw_rate = 0.1* (Quat2Angle(dest[RTH_index].orientation).z - Quat2Angle(pix_local.pose.pose.orientation).z);
		local_setpoint_raw_pub.publish(vel_target);
	}

	if(arrival_check(pix_local, dest[RTH_index]))
	{
		// log
		loglog = "RTH waypoint " + std::to_string(RTH_index) + " arrived among " + std::to_string(height_step+1) + " number of waypoints";
		log_pub_ros_info(LOG_IMPORTANT,loglog);

		RTH_index++;
		
		if (RTH_index == height_step+1)
		{
			double landing_old;
	    	double landing_cur;
	    	ros::Time landing_time;
			while(pix_extstate.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
			{

				timeout_check();
				return_target.type_mask = velocity_control_type;
				return_target.velocity.x = vel_saturation(-0.3 * pix_local.pose.pose.position.x);
		        return_target.velocity.y = vel_saturation(-0.3 * pix_local.pose.pose.position.y);
		        return_target.velocity.z = (-1)*landing_vel;
		        if(ros::Time::now() - landing_time > ros::Duration(4.0)){
		            landing_cur = pix_local.pose.pose.position.z; // Change current height value
		            landing_time = ros::Time::now();

		            // landing clear condition
		            // old height value > cur height value and the difference is less than 50cm
		            if((landing_old > landing_cur) && abs(landing_old - landing_cur) < 0.5){
		                if(pix_state.armed){
			                mavros_msgs::CommandBool cmd;
			                cmd.request.value = false;
			                cmdArming.call(cmd);

		                    // log
		                    loglog = "Landing on going";
		                    log_pub_ros_info(LOG_ERROR,loglog);
		                }
		            }
		            landing_old = landing_cur; // Change old height value
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
	}
	// else
	// {
	// 	local_setpoint_raw_pub.publish(RTHtarget[RTH_index]);
	// }
	ros::Duration(0.05).sleep();
}

void UAV_control::spinOnce()
{
	if(mission_started == true)
	{
		timeout_check();
		if(!return_home)
		{
			if(!timeout_flag){
				if (!home_registered) home_registered = register_home();
				else if (!takeoff_flag) takeoff();
				else if (!arrival_flag) move2waypoint();
				else if (!follower_start) hover();
				else follow();
			}
			else RTH();
		}
		else RTH();
	}
	else
	{
		mission_start_time = ros::Time::now();
		// log
	    loglog = "mission not started";
	    log_pub_ros_info(LOG_ERROR,loglog);

	}
}
