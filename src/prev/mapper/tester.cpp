#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <thread>
#include <sstream>
#include <queue>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

// #include <n_cpp/points_node.h>
// #include <n_cpp/points_nodes.h>
// #include <n_cpp/path_terminate.h>
// #include <n_cpp/local_path.h>
// #include <n_cpp/global_path.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/pcl_config.h>

#include <tf/transform_broadcaster.h>

nav_msgs::Path pathpath;
geometry_msgs::PoseStamped posepose;

Eigen::Matrix4f map_t_body, body_t_cam;

pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_pc_accumulated (new pcl::PointCloud<pcl::PointXYZ>());


Eigen::Matrix4f front_static_trans = Eigen::Matrix4f::Identity();

void set_trans(){
  tf::Matrix3x3 static_m;
  static_m.setRPY(1.570796, -3.141592, 1.570796);
  front_static_trans(0,0) = static_m[0][0];
  front_static_trans(0,1) = static_m[0][1];
  front_static_trans(0,2) = static_m[0][2];
  front_static_trans(1,0) = static_m[1][0];
  front_static_trans(1,1) = static_m[1][1];
  front_static_trans(1,2) = static_m[1][2];
  front_static_trans(2,0) = static_m[2][0];
  front_static_trans(2,1) = static_m[2][1];
  front_static_trans(2,2) = static_m[2][2];
  front_static_trans(3,3) = 1;
  front_static_trans(0,3) = 0.1;
  front_static_trans(1,3) = 0;
  front_static_trans(2,3) = 0;
}

geometry_msgs::Vector3 Quat2Angle(geometry_msgs::Quaternion quat)
{
    geometry_msgs::Vector3 res;
    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(res.x, res.y, res.z);
    return res;
}

Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose)
{
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
  tf::Matrix3x3 m(q);
  result(0,0) = m[0][0];
  result(0,1) = m[0][1];
  result(0,2) = m[0][2];
  result(1,0) = m[1][0];
  result(1,1) = m[1][1];
  result(1,2) = m[1][2];
  result(2,0) = m[2][0];
  result(2,1) = m[2][1];
  result(2,2) = m[2][2];
  result(3,3) = 1;

  result(0,3) = geoPose.position.x;
  result(1,3) = geoPose.position.y;
  result(2,3) = geoPose.position.z;

  return result;
}

Eigen::Matrix4f interpolation(geometry_msgs::PoseStamped pose_previous, geometry_msgs::PoseStamped pose_current, double points_time){
  double prev_pose_time = pose_previous.header.stamp.toSec();
  double curr_pose_time = pose_current.header.stamp.toSec();

  // double ratio = 1;
  double ratio = (points_time - prev_pose_time)/(curr_pose_time - prev_pose_time);
  std::cout << "ratio : " << ratio << std::endl;

  tf::Quaternion q_interpolated;
  tf::Quaternion q0(pose_previous.pose.orientation.x, pose_previous.pose.orientation.y, pose_previous.pose.orientation.z, pose_previous.pose.orientation.w);
  q0.normalize();
  tf::Quaternion q1(pose_current.pose.orientation.x, pose_current.pose.orientation.y, pose_current.pose.orientation.z, pose_current.pose.orientation.w);
  q1.normalize();
  double dot = q0.dot(q1);

  // std::cout << "q0 : " << q0.getX() << ", " << q0.getY() << ", " << q0.getZ() << ", " << q0.getW() << std::endl;
  // std::cout << "q1 : " << q1.getX() << ", " << q1.getY() << ", " << q1.getZ() << ", " << q1.getW() << std::endl;

  if (dot < 0.0f) {
        q1 = q1 * (-1);
        dot = - dot;
  }

  const double DOT_THRESHOLD = 0.999;

  if (dot > DOT_THRESHOLD) {
    q_interpolated = q0 + (q1 - q0) * ratio;

  }
  else{
    // Since dot is in range [0, DOT_THRESHOLD], acos is safe
    q_interpolated = q0.slerp(q1, ratio);
    // std::cout << "q_interpolated : " << q_interpolated.getX() << ", " << q_interpolated.getY() << ", " << q_interpolated.getZ() << ", " << q_interpolated.getW() << std::endl;
  }
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  tf::Matrix3x3 m(q_interpolated);
  result(0,0) = m[0][0];
  result(0,1) = m[0][1];
  result(0,2) = m[0][2];
  result(1,0) = m[1][0];
  result(1,1) = m[1][1];
  result(1,2) = m[1][2];
  result(2,0) = m[2][0];
  result(2,1) = m[2][1];
  result(2,2) = m[2][2];
  result(3,3) = 1;

  result(0,3) = pose_previous.pose.position.x + (pose_current.pose.position.x - pose_previous.pose.position.x) * ratio;
  result(1,3) = pose_previous.pose.position.y + (pose_current.pose.position.y - pose_previous.pose.position.y) * ratio;
  result(2,3) = pose_previous.pose.position.z + (pose_current.pose.position.z - pose_previous.pose.position.z) * ratio;

  std::cout << "x : " << result(0,3) << ", y : " << result(1,3) << ". z : " << result(2,3) << std::endl;
  geometry_msgs::PoseStamped path_temp;
  path_temp.header.frame_id = "map";
  path_temp.pose.position.x = result(0,3);
  path_temp.pose.position.y = result(1,3);
  path_temp.pose.position.z = result(2,3);
  path_temp.pose.orientation.x = q_interpolated.getX();
  path_temp.pose.orientation.y = q_interpolated.getY();
  path_temp.pose.orientation.z = q_interpolated.getZ();
  path_temp.pose.orientation.x = q_interpolated.getW();
  pathpath.poses.push_back(path_temp);

  return result;
}

void cb_points_front(const sensor_msgs::PointCloud2::ConstPtr& msg){
  static ros::Time last_request;
  std::cout << "last_request : " << last_request.toSec() << std::endl;
  ros::Duration diff = ros::Time::now() - last_request;
  std::cout << "diff : " << diff.toSec() << std::endl;
  
  if (diff > ros::Duration(0.2)){
    std::cout << "NOW!!!!" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr tran_pc (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloudIn);
    Eigen::Matrix4f map_t_cam = map_t_body * body_t_cam ;
    pcl::PassThrough<pcl::PointXYZ> full_filter;
    full_filter.setInputCloud(cloudIn);
    full_filter.setFilterFieldName("z");
    full_filter.setFilterLimits(0.8, 12.0);
    full_filter.filter(*cloudOut);
    // Eigen::Matrix4f trans, transtrans;
    posepose.header.frame_id = "map";
    pathpath.poses.push_back(posepose);
    // transtrans = geoPose2eigen(posepose.pose);
    // trans = transtrans * front_static_trans;
    // pcl::transformPointCloud(*cloudOut, *tran_pc, trans);
    pcl::transformPointCloud(*cloudOut, *tran_pc, map_t_cam);
    *ptr_pc_accumulated += *tran_pc;
    last_request = ros::Time::now();
    cloudIn->clear();
    cloudOut->clear();
    tran_pc->clear();
  }
}

// void cb_model_state(const gazebo_msgs::ModelStates::ConstPtr& msg){
//   int iris_index = -1;
//   static int seq_value = 0; 
//   for (int i = 0; i < msg->name.size(); ++i){
//     if(msg->name[i] == "iris_front_up_depth_camera") iris_index = i; 
//   }
//   if(iris_index != -1){
//     geometry_msgs::PoseStamped input;
//     input.header.seq = ++seq_value;
//     input.header.stamp = ros::Time::now();
//     input.header.frame_id = "map";
//     input.pose = msg->pose[iris_index];
//     // pose_queue.push(input);
//     posepose = input;
//   }
// }

void cb_tf(const tf2_msgs::TFMessage::ConstPtr& msg){
  for (int l=0; l < msg->transforms.size(); l++){
    if (msg->transforms[l].header.frame_id=="map" && msg->transforms[l].child_frame_id=="base_link"){
      tf::Quaternion q(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m(q);
      map_t_body(0,0) = m[0][0];
      map_t_body(0,1) = m[0][1];
      map_t_body(0,2) = m[0][2];
      map_t_body(1,0) = m[1][0];
      map_t_body(1,1) = m[1][1];
      map_t_body(1,2) = m[1][2];
      map_t_body(2,0) = m[2][0];
      map_t_body(2,1) = m[2][1];
      map_t_body(2,2) = m[2][2];

      map_t_body(0,3) = msg->transforms[l].transform.translation.x;
      map_t_body(1,3) = msg->transforms[l].transform.translation.y;
      map_t_body(2,3) = msg->transforms[l].transform.translation.z;
      map_t_body(3,3) = 1.0;
    }
  }
}


void cb_st_tf(const tf2_msgs::TFMessage::ConstPtr& msg){
  for (int l=0; l < msg->transforms.size(); l++){
    if (msg->transforms[l].header.frame_id=="base_link" && msg->transforms[l].child_frame_id=="front_camera_link"){
      tf::Quaternion q(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m(q);
      body_t_cam(0,0) = m[0][0];
      body_t_cam(0,1) = m[0][1];
      body_t_cam(0,2) = m[0][2];
      body_t_cam(1,0) = m[1][0];
      body_t_cam(1,1) = m[1][1];
      body_t_cam(1,2) = m[1][2];
      body_t_cam(2,0) = m[2][0];
      body_t_cam(2,1) = m[2][1];
      body_t_cam(2,2) = m[2][2];

      body_t_cam(0,3) = msg->transforms[l].transform.translation.x;
      body_t_cam(1,3) = msg->transforms[l].transform.translation.y;
      body_t_cam(2,3) = msg->transforms[l].transform.translation.z;
      body_t_cam(3,3) = 1.0;
    }
  }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tester");
    ros::NodeHandle n;
    ros::Publisher result_pub = n.advertise<sensor_msgs::PointCloud2>("result", 1000);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("result_path", 1000);

    ros::Subscriber points_sub_front = n.subscribe("/camera_front/depth/points", 1000, cb_points_front);
    // ros::Subscriber sub_UAV = n.subscribe("/UAV", 1000, cb_model_state);
    // ros::Subscriber sub_model = n.subscribe("/gazebo/model_states", 1000, cb_model_state);
    ros::Subscriber sub_tf = n.subscribe("/tf", 1000, cb_tf);
    ros::Subscriber sub_st_tf = n.subscribe("/tf_static", 1000, cb_st_tf);
    

    ros::Rate loop_rate(20);

    set_trans();
    std::cout << "Let's getting to rumble!" << std::endl;
      
    while(ros::ok()){
      
      sensor_msgs::PointCloud2 pub_points;
      pcl::PointCloud<pcl::PointXYZ> cloud_dst;
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      double var_voxelsize = 0.05;
      voxel_filter.setInputCloud(ptr_pc_accumulated);
      voxel_filter.setLeafSize(var_voxelsize, var_voxelsize, var_voxelsize);
      voxel_filter.filter(cloud_dst);
      pcl::toROSMsg<pcl::PointXYZ>(cloud_dst, pub_points);
      *ptr_pc_accumulated = cloud_dst;

      pub_points.header.frame_id = "map";
      pathpath.header.frame_id = "map";

      result_pub.publish(pub_points);
      path_pub.publish(pathpath);
      
      loop_rate.sleep();
      ros::spinOnce();
    }

    return 0;
}
