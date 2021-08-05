#include "control.hpp"

void UAV_control::cb_pix_state(const mavros_msgs::State::ConstPtr& msg){pix_state = *msg;}
void UAV_control::cb_pix_extstate(const mavros_msgs::ExtendedState::ConstPtr& msg){pix_extstate = *msg;}
void UAV_control::cb_pix_local(const nav_msgs::Odometry::ConstPtr &msg){pix_local = *msg;}
// void UAV_control::cb_pix_GPSlocal(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){pix_GPSlocal = *msg;}
void UAV_control::cb_pix_altitude(const mavros_msgs::Altitude::ConstPtr &msg){pix_altitude = *msg;}
void UAV_control::cb_pix_home(const mavros_msgs::HomePosition::ConstPtr &msg)
{
	pix_home = *msg;
	trans_home_quat.x = 0;
    trans_home_quat.y = 0;
    trans_home_quat.z = pix_home.orientation.y;
    trans_home_quat.w = pix_home.orientation.x;
}
void UAV_control::cb_tracker_result(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
	tracker_result = *msg;
	last_detected = ros::Time::now();
}
// void UAV_control::cb_detection_tracking(const std_msgs::Bool::ConstPtr &msg){detection_tracking = msg->data;}
// bool UAV_control::fn_mission_start(drone_contest::mission_start::Request &request,
// 					  			   drone_contest::mission_start::Response &response)
bool UAV_control::fn_mission_start(jane_ui::automatic_mission_start::Request &request,
					  			   jane_ui::automatic_mission_start::Response &response)
{
	if(mission_started == false)
	{
		mission_started = true;
		response.result = true;
		std::cout << "waypoint_x :" << waypoint_x <<std::endl;
		std::cout << "waypoint_y :" << waypoint_y <<std::endl;
		std::cout << "timeout :" << timeout <<std::endl;
	}
	else
	{
		// log
	    loglog ="already on mission";
	    log_pub_ros_info(LOG_ERROR,loglog);		
		response.result = false;
	}
}

bool UAV_control::fn_return_home(jane_ui::return_home::Request &request,
                    			 jane_ui::return_home::Response &response)
{
	if(return_home == false)
	{
		// log
		loglog = "RTH activated";
		log_pub_ros_info(LOG_IMPORTANT,loglog);
		return_home = true;
		response.result = true;	
	}	
	else
	{
		// log
        loglog = "Already returning to home. Please wait";
        log_pub_ros_info(LOG_ERROR,loglog);
	}
	
}
// bool UAV_control::fn_follower_start(drone_contest::follower_start::Request &request,
//                        				drone_contest::follower_start::Response &response)
bool UAV_control::fn_follower_start(jane_ui::next_mission::Request &request,
                       				jane_ui::next_mission::Response &response)
{
	// log
	loglog = "Follower mode started";
	log_pub_ros_info(LOG_IMPORTANT,loglog);
	// safe_distance = request.safe_distance;
	follower_start = true;
	response.result = true;	
}
