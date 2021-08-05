#include "path_gen.hpp"
bool N_path::next_step(n_cpp::NextStep::Request &req,
               		   n_cpp::NextStep::Response &res)
{
	// n_cpp::get_octomap map_data;
	// map_data.request.pub_pc = pub_octomap;
	// map_data.request.terminate_flag = false;
	// client_get_octomap.call(map_data);

	n_cpp::get_triple_points map_data;
	map_data.request.pub_pc = pub_octomap;
	map_data.request.terminate_flag = false;
	client_triplepoints.call(map_data);

	// octo_new = map_data.response.octo_change;
	tripoints = map_data.response.tripoints;
	octo_pose = map_data.response.curr_pose;

	nav_msgs::Path raw_path;
	n_cpp::NextStep::Response result;

	// path = path_gen();
	result = new_path_gen(req.escape);
	raw_path = result.raw_path;
	raw_path.header = curr_pose.header;
	raw_path.header.frame_id = "map";
	path_pub.publish(raw_path);

	res = result;

	if(pub_candidate==true){
		// visualization_msgs::MarkerArray normals_marker;
		// visualization_msgs::MarkerArray candidate_marker;
		// current_position

		//DUCK
		normal_pub.publish(normals_marker);
		// cand_pub.publish();
	}

	return true;
}
bool N_path::mission_terminate(n_cpp::mission_terminate::Request &req,
                    		   n_cpp::mission_terminate::Response &res)
{
	res.coverage_time.data = ros::Time::now() - init_time;

	n_cpp::get_octomap map_data;
	map_data.request.terminate_flag = true;
	client_get_octomap.call(map_data);

	return true;
}