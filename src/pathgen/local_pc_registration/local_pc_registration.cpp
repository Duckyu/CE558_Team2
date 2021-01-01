#include "local_pc_registration.hpp"

void exploration_local_map::cb_points_front(const sensor_msgs::PointCloud2::ConstPtr& msg){
  front_ = msg2node(*msg);
}

void exploration_local_map::cb_points_up(const sensor_msgs::PointCloud2::ConstPtr& msg){
  up_ = msg2node(*msg);
}

void exploration_local_map::cb_model_state(const geometry_msgs::PoseStamped::ConstPtr& msg){
  pose_previous = pose_current; 
  pose_current = *msg;
}

bool exploration_local_map::fn_path_terminate(n_cpp::path_terminate::Request &request,
                       n_cpp::path_terminate::Response &response){
  termination_flag = true;
}

n_cpp::points_node exploration_local_map::msg2node(sensor_msgs::PointCloud2 points,
                                                         geometry_msgs::PoseStamped pose_previous,
                                                         geometry_msgs::PoseStamped pose_current){
  double points_time = points.header.stamp.toSec();
  double prev_pose_time = pose_previous.header.stamp.toSec();
  double curr_pose_time = pose_current.header.stamp.toSec();
  n_cpp::points_node pointsNode;
//////////////////////
  pcl::PointCloud<pcl::PointXYZ> cloud_dst;
  pcl::fromROSMsg(points, cloud_dst);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  *cloud = cloud_dst;

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 10.0);
  // pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  pointsNode.node_sensor = *cloud_filtered;
  pointsNode.node_pose.position.x = pose_previous.pose.position.x + (points_time - prev_pose_time)/(curr_pose_time - prev_pose_time) * (pose_current.pose.position.x - pose_previous.pose.position.x); 
  pointsNode.node_pose.position.y = pose_previous.pose.position.y + (points_time - prev_pose_time)/(curr_pose_time - prev_pose_time) * (pose_current.pose.position.y - pose_previous.pose.position.y);
  pointsNode.node_pose.position.z = pose_previous.pose.position.z + (points_time - prev_pose_time)/(curr_pose_time - prev_pose_time) * (pose_current.pose.position.z - pose_previous.pose.position.z);
  pointsNode.node_pose.orientation = pose_current.pose.orientation;
  return pointsNode;
}

pcl::PointCloud<pcl::PointXYZ> exploration_local_map::transform(n_cpp::points_node node){
  // try{
  //   listener.lookupTransform("/map", "/base_link",
  //                             ros::Time(0), uav_transform);
  //   listener.lookupTransform("/base_link", "/up_camera_link",
  //                             ros::Time(0), upCam_transform);
  //   listener.lookupTransform("/base_link", "/front_camera_link",
  //                             ros::Time(0), frontCam_transform);
  // }
  // catch (tf::TransformException &ex) {
  //   ROS_ERROR("%s",ex.what());
  //   ros::Duration(1.0).sleep();
  // }
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Matrix4f trans;
  trans = cvt::geoPose2eigen(node.node_pose);
  pcl::transformPointCloud(node.node_sensor, *ptr_transformed, trans);
  return *ptr_transformed;
}

void exploration_local_map::add(n_cpp::points_node node){
  pc_transformed = transform(node);
  pc_accumulated += pc_transformed;
}


pcl::PointCloud<pcl::PointXYZ> exploration_local_map::voxelize(pcl::PointCloud<pcl::PointXYZ> pc){
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  *cloud = pc;
  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (2.5981f, 3.4641f, 2.5981f);
  sor.filter (*cloud_filtered);
  return *cloud_filtered;
}


pcl::PointCloud<pcl::Normal> exploration_local_map::normalVectorEstimate(pcl::PointCloud<pcl::PointXYZ> pc){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
  *cloud = pc;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  return *cloud_normals;
}
void exploration_local_map::samplingBasePathGen(){
  
  pc_accumulated = voxelize(pc_accumulated);
  pcl::PointCloud<pcl::Normal> norm;
  norm = normalVectorEstimate(pc_accumulated);
  int norm_size = norm.size();
  for(int i = 0; i < norm_size/20; ++i){
    geometry_msgs::PoseStamped temp;
    temp.pose.position.x = norm.points[20*i+10].x + norm.points[20*i+10].n_x * 2.0;
    temp.pose.position.y = norm.points[20*i+10].y + norm.points[20*i+10].n_y * 2.0;
    temp.pose.position.z = norm.points[20*i+10].z + norm.points[20*i+10].n_z * 2.0;
    tf::Quaternion q;
    q.setRPY(0,0,atan(norm.points[20*i+10].nx/norm.points[20*i+10].ny));
    temp.pose.orientation.x = q.getX();
    temp.pose.orientation.y = q.getY();
    temp.pose.orientation.z = q.getZ();
    temp.pose.orientation.w = q.getW();
    pose_path.push_back(temp);
  }  
}
// void exploration_local_map::checkCollision(geometry_msgs::PoseStamped poses[], octomap::OcTree octomap);
void exploration_local_map::pubPath(){
  // checkCollision()
  termination_flag = false;
  last_pc_added = ros::Time::now();
  n_cpp::local_path path;
  path.request.path_size = poses.size();
  path.request.poses = pose_path;
  srv_local_path.call(path);
  previous_pc_accumulated.clear();
  previous_pc_accumulated = pc_accumulated;
  pose_path.clear();
  pc_accumulated.clear();
  pc_transformed.clear();
  normal_points.clear();
  normal_voxeled_points.clear();
}

void exploration_local_map::spinOnce(){

  tf::TransformListener listener;
  tf::StampedTransform uav_transform;
  tf::StampedTransform upCam_transform;
  tf::StampedTransform frontCam_transform;


  if (!termination_flag){
    add(front_);
    add(up_);
  }
  else{
    samplingBasePathGen();
    pubPath(pose_path);
  }
}





////////////////////Eigen::Vector4f centroid; pcl::compute3DCentroid (*cloud, centroid);










// ///passthrough.cpp
// int main (int argc, char** argv)
// {
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

// pcl::PassThrough<pcl::PointXYZ> pass;
// pass.setInputCloud (cloud);
// pass.setFilterFieldName ("z");
// pass.setFilterLimits (0.0, 1.0);
// // pass.setFilterLimitsNegative (true);
// pass.filter (*cloud_filtered);
//   // Fill in the cloud data
//   cloud->width  = 5;
//   cloud->height = 1;
//   cloud->points.resize (cloud->width * cloud->height);

//   for (auto& point: *cloud)
//   {
//     point.x = 1024 * rand () / (RAND_MAX + 1.0f);
//     point.y = 1024 * rand () / (RAND_MAX + 1.0f);
//     point.z = 1024 * rand () / (RAND_MAX + 1.0f);
//   }

//   std::cerr << "Cloud before filtering: " << std::endl;
//   for (const auto& point: *cloud)
//     std::cerr << "    " << point.x << " "
//                         << point.y << " "
//                         << point.z << std::endl;

//   // Create the filtering object
//   pcl::PassThrough<pcl::PointXYZ> pass;
//   pass.setInputCloud (cloud);
//   pass.setFilterFieldName ("z");
//   pass.setFilterLimits (0.0, 1.0);
//   // pass.setFilterLimitsNegative (true);
//   pass.filter (*cloud_filtered);
