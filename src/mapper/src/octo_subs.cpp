#include "octomap.hpp"

void N_octomap::subs_front(const sensor_msgs::PointCloud2::ConstPtr& msg){_front = *msg;}
void N_octomap::subs_up(const sensor_msgs::PointCloud2::ConstPtr& msg){_up = *msg;}
void N_octomap::subs_pose(const geometry_msgs::PoseStamped::ConstPtr& msg){
  curr_pose = *msg;
  ////////////If cb_tf exists, kill below code //////////////
  Eigen::Quaternionf q(msg->pose.orientation.w,
                       msg->pose.orientation.x,
                       msg->pose.orientation.y,
                       msg->pose.orientation.z);
  map_t_body.block(0,0,3,3) = q.normalized().toRotationMatrix();
  map_t_body(0,3) = msg->pose.position.x;
  map_t_body(1,3) = msg->pose.position.y;
  map_t_body(2,3) = msg->pose.position.z;
  //////////////////////////////////////////////////////////
}

// void N_octomap::cb_tf(const tf2_msgs::TFMessage::ConstPtr& msg){
//   for (int l=0; l < msg->transforms.size(); l++){
//     if (msg->transforms[l].header.frame_id=="map" && msg->transforms[l].child_frame_id=="base_link"){
    	// Eigen::Quaternionf q(msg->transforms[l].transform.rotation.w,
    	// 					           msg->transforms[l].transform.rotation.x,
  			// 			             msg->transforms[l].transform.rotation.y,
    	// 					           msg->transforms[l].transform.rotation.z);
//     	map_t_body.block(0,0,3,3) = q.normalized().toRotationMatrix();

// 		map_t_body(0,3) = msg->transforms[l].transform.translation.x;
// 		map_t_body(1,3) = msg->transforms[l].transform.translation.y;
// 		map_t_body(2,3) = msg->transforms[l].transform.translation.z;
//     }
//   }
// }

void N_octomap::cb_st_tf(const tf2_msgs::TFMessage::ConstPtr& msg){
  for (int l=0; l < msg->transforms.size(); l++){
    if (msg->transforms[l].header.frame_id=="base_link" && msg->transforms[l].child_frame_id=="front_camera_link"){
      tf::Quaternion q(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m(q);
      body_t_front(0,0) = m[0][0];
      body_t_front(0,1) = m[0][1];
      body_t_front(0,2) = m[0][2];
      body_t_front(1,0) = m[1][0];
      body_t_front(1,1) = m[1][1];
      body_t_front(1,2) = m[1][2];
      body_t_front(2,0) = m[2][0];
      body_t_front(2,1) = m[2][1];
      body_t_front(2,2) = m[2][2];

      body_t_front(0,3) = msg->transforms[l].transform.translation.x;
      body_t_front(1,3) = msg->transforms[l].transform.translation.y;
      body_t_front(2,3) = msg->transforms[l].transform.translation.z;
      body_t_front(3,3) = 1.0;

      // std::cout << "body_t_front: " << std::endl;
      // std::cout << body_t_front << std::endl;
    }
    
    if (msg->transforms[l].header.frame_id=="base_link" && msg->transforms[l].child_frame_id=="up_camera_link"){
      tf::Quaternion q(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m(q);
      body_t_up(0,0) = m[0][0];
      body_t_up(0,1) = m[0][1];
      body_t_up(0,2) = m[0][2];
      body_t_up(1,0) = m[1][0];
      body_t_up(1,1) = m[1][1];
      body_t_up(1,2) = m[1][2];
      body_t_up(2,0) = m[2][0];
      body_t_up(2,1) = m[2][1];
      body_t_up(2,2) = m[2][2];

      body_t_up(0,3) = msg->transforms[l].transform.translation.x;
      body_t_up(1,3) = msg->transforms[l].transform.translation.y;
      body_t_up(2,3) = msg->transforms[l].transform.translation.z;
      body_t_up(3,3) = 1.0;
      // std::cout << "body_t_up: " << std::endl;
      // std::cout << body_t_up << std::endl;
    }
  }
}