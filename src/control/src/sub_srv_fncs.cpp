#include "control.hpp"

void UAV_control::cb_pix_state(const mavros_msgs::State::ConstPtr& msg){pix_state = *msg;}
void UAV_control::cb_pix_extstate(const mavros_msgs::ExtendedState::ConstPtr& msg){pix_extstate = *msg;}
void UAV_control::cb_pix_local(const nav_msgs::Odometry::ConstPtr &msg){pix_local = *msg;}
// void UAV_control::cb_pix_GPSlocal(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){pix_GPSlocal = *msg;}
void UAV_control::cb_pix_altitude(const mavros_msgs::Altitude::ConstPtr &msg){pix_altitude = *msg;}
void UAV_control::cb_pix_home(const mavros_msgs::HomePosition::ConstPtr &msg){pix_home = *msg;}
void UAV_control::cb_tracker_result(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg){tracker_result = *msg;}

bool UAV_control::fn_mission_start(drone_contest::mission_start::Request &request,
					  			   drone_contest::mission_start::Response &response)
{
	if(mission_started == false)
	{
		if(request.mission_local == false)
		{
			mission_started = true;
			mission_local = false;
			waypoint_longitude = request.waypoint_longitude;
			waypoint_latitude = request.waypoint_latitude;
			timeout = request.timeout;
			response.result = true;
		}
		else
		{
			mission_started = true;
			mission_local = true;
			waypoint_x = request.waypoint_x;
			waypoint_y = request.waypoint_y;
			timeout = request.timeout;
			response.result = true;				
		}
		std::cout << "timeout" << std::to_string(timeout) <<std::endl;
	}
	else
	{
		// log
	    loglog ="already on mission";
	    log_pub_ros_info(LOG_ERROR,loglog);		
		response.result = false;
	}
}

bool UAV_control::fn_return_home(drone_contest::return_home::Request &request,
                    			 drone_contest::return_home::Response &response)
{
	if(response.result == false)
	{
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
bool UAV_control::fn_follower_start(drone_contest::follower_start::Request &request,
                       				drone_contest::follower_start::Response &response)
{
	safe_distance = request.safe_distance;
	response.result = true;	
	// log
	loglog = "Follower mode started";
	log_pub_ros_info(LOG_IMPORTANT,loglog);
}