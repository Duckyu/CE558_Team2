#include "control.hpp"

void exploration_control::cb_pix_state(const mavros_msgs::State::ConstPtr& msg){pix_state = *msg;}
void exploration_control::cb_pix_extstate(const mavros_msgs::ExtendedState::ConstPtr& msg){pix_extstate = *msg;}
void exploration_control::cb_pix_local(const nav_msgs::Odometry::ConstPtr &msg){pix_local = *msg;}
void exploration_control::cb_pix_home(const mavros_msgs::HomePosition::ConstPtr &msg)
{
	pix_home = *msg;
	trans_home_quat.x = 0;
    trans_home_quat.y = 0;
    trans_home_quat.z = pix_home.orientation.y;
    trans_home_quat.w = pix_home.orientation.x;
}
bool exploration_control::fn_mission_start(n_cpp::mission_start::Request &request,
					  			   		   n_cpp::mission_start::Response &response)
{
	if(mission_started == false)
	{
		mission_started = true;
		response.result = true;
	}
	else
	{
		// log
	    loglog ="already on mission";
	    log_pub_ros_info(LOG_ERROR,loglog);		
		response.result = false;
	}
}

bool exploration_control::fn_init_step(n_cpp::init_step::Request &request,
                  					   n_cpp::init_step::Response &response){
	init_radius = request.init_radius;
	control_mode = initiation;
	response.result = true;
}
bool exploration_control::fn_local_path(n_cpp::local_path::Request &request,
                   						n_cpp::local_path::Response &response){
	// log
    loglog ="local path received";
    log_pub_ros_info(LOG_ERROR,loglog);

    control_mode = local_control;
	
	response.result = false;
	path_size = request.path_size;
	received_path = request.local_path;
	response.result = true;
}
bool exploration_control::fn_global_path(n_cpp::global_path::Request &request,
                   						 n_cpp::global_path::Response &response){
	path_size = request.path_size;
	received_path = request.global_path;
	response.result = true;
}
bool exploration_control::fn_end(n_cpp::end::Request &request,
            					 n_cpp::end::Response &response){

}