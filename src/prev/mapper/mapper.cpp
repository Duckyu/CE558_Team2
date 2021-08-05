#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <sstream>

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
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <n_cpp/points_node.h>
#include <n_cpp/points_nodes.h>
#include <n_cpp/path_terminate.h>
#include <n_cpp/local_path.h>
#include <n_cpp/global_path.h>

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

#include <unavlib/convt.h>
#include <unavlib/others.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_pc_accumulated(new pcl::PointCloud<pcl::PointXYZ>);
// sensor_msgs::PointCloud2 test;
n_cpp::points_node up_;
n_cpp::points_node front_;
geometry_msgs::PoseStamped pose_previous, pose_current;



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



n_cpp::points_node msg2node(sensor_msgs::PointCloud2 points,
                                  geometry_msgs::PoseStamped pose_previous,
                                  geometry_msgs::PoseStamped pose_current){
  double points_time = points.header.stamp.toSec();
  double prev_pose_time = pose_previous.header.stamp.toSec();
  double curr_pose_time = pose_current.header.stamp.toSec();
  n_cpp::points_node pointsNode;
////////////////////////
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_dst (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(points, *ptr_cloud_dst);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (ptr_cloud_dst);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 10.0);
  // pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  pcl::toROSMsg(*cloud_filtered, pointsNode.node_sensor);
  pointsNode.node_sensor.header.frame_id = points.header.frame_id;
  pointsNode.node_pose.position.x = pose_previous.pose.position.x + (points_time - prev_pose_time)/(curr_pose_time - prev_pose_time) * (pose_current.pose.position.x - pose_previous.pose.position.x); 
  pointsNode.node_pose.position.y = pose_previous.pose.position.y + (points_time - prev_pose_time)/(curr_pose_time - prev_pose_time) * (pose_current.pose.position.y - pose_previous.pose.position.y);
  pointsNode.node_pose.position.z = pose_previous.pose.position.z + (points_time - prev_pose_time)/(curr_pose_time - prev_pose_time) * (pose_current.pose.position.z - pose_previous.pose.position.z);
  geometry_msgs::Vector3 tmep_previous;
  geometry_msgs::Vector3 tmep_current;
  tmep_previous = Quat2Angle(pose_previous.pose.orientation);
  tmep_current = Quat2Angle(pose_current.pose.orientation);
  tf::Quaternion q;
  q.setRPY((tmep_previous.x+tmep_current.x)/2.0,
           (tmep_previous.y+tmep_current.y)/2.0,
           (tmep_previous.z+tmep_current.z)/2.0);
  q.normalize();

  pointsNode.node_pose.orientation.x = q.getX();
  pointsNode.node_pose.orientation.y = q.getY();
  pointsNode.node_pose.orientation.z = q.getZ();
  pointsNode.node_pose.orientation.w = q.getW();


  return pointsNode;
}

void cb_points_front(const sensor_msgs::PointCloud2::ConstPtr& msg){
  front_ = msg2node(*msg, pose_previous, pose_current);
  // std::cout << front_.node_pose.position.x <<std::endl;
  // std::cout << front_.node_pose.position.y <<std::endl;
  // std::cout << front_.node_pose.position.z <<std::endl;
  // std::cout << front_.node_pose.orientation.x <<std::endl;
  // std::cout << front_.node_pose.orientation.y <<std::endl;
  // std::cout << front_.node_pose.orientation.z <<std::endl;
  // std::cout << front_.node_pose.orientation.w <<std::endl;
}
void cb_points_up(const sensor_msgs::PointCloud2::ConstPtr& msg){
  up_ = msg2node(*msg, pose_previous, pose_current);
  // test = *msg;
  // std::cout << test.header.frame_id << std::endl;
}
void cb_model_state(const geometry_msgs::PoseStamped::ConstPtr& msg){
  pose_previous = pose_current; 
  pose_current = *msg;
}  


pcl::PointCloud<pcl::PointXYZ> transform(n_cpp::points_node node, int direction){
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZ>());

  Eigen::Matrix4f trans;
  Eigen::Matrix4f st_trans;
  trans = geoPose2eigen(node.node_pose);
  pcl::fromROSMsg(node.node_sensor, *ptr_cloud);
  if(direction == 0){
    geometry_msgs::Pose up_pose;
    up_pose.position.x = 0.0;
    up_pose.position.y = 0.0;
    up_pose.position.z = 0.05;
    up_pose.orientation.x = 0.0;
    up_pose.orientation.y = 0.0;
    up_pose.orientation.z = -0.707;
    up_pose.orientation.w = 0.707;
    st_trans = geoPose2eigen(up_pose);
  }
  else if(direction == 1){
    geometry_msgs::Pose front_pose;
    front_pose.position.x = 0.1;
    front_pose.position.y = 0.0;
    front_pose.position.z = 0.0;
    front_pose.orientation.x = 0.5;
    front_pose.orientation.y = -0.5;
    front_pose.orientation.z = 0.5;
    front_pose.orientation.w = -0.5;
    st_trans = geoPose2eigen(front_pose);
  }

  
  pcl::transformPointCloud(*ptr_cloud, *ptr_transformed, st_trans, true);
  pcl::transformPointCloud(*ptr_transformed, *ptr_transformed, trans, true);


  return *ptr_transformed;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "global_mapGen");
    ros::NodeHandle n;
    ros::Publisher result_pub = n.advertise<sensor_msgs::PointCloud2>("result", 1000);



    ros::Subscriber points_sub_front = n.subscribe("/camera_front/depth/points", 1000, cb_points_front);
    ros::Subscriber points_sub_up = n.subscribe("/camera_up/depth/points", 1000, cb_points_up);
    ros::Subscriber sub_UAV = n.subscribe("/UAV", 1000, cb_model_state);
    ros::Rate loop_rate(10);
    ros::Time registerTime = ros::Time::now();
    while(ros::ok()){
      if (ros::Time::now() - registerTime > ros::Duration(0.1)){
        pcl::PointCloud<pcl::PointXYZ> pc_accumulated;
        pc_accumulated = *ptr_pc_accumulated;
        pc_accumulated += transform(up_, 0);
        pc_accumulated += transform(front_, 1);
        std::vector<int> index;
        pcl::removeNaNFromPointCloud(pc_accumulated, pc_accumulated, index);
        *ptr_pc_accumulated = pc_accumulated;
        // ROS_ERROR("DEBUG");
        // simpleVis(ptr_pc_accumulated);
        // pcl::PointCloud<pcl::PointXYZ> final_points;
        sensor_msgs::PointCloud2 pub_points;
        // final_points = *ptr_pc_accumulated;
        // pcl::toROSMsg(final_points, pub_points);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_temp(new pcl::PointCloud<pcl::PointXYZ>());
        *ptr_temp = transform(up_, 0);
        std::cout <<  "SIZE: " << ptr_temp->points.size() <<std::endl;
        pcl::toROSMsg(*ptr_temp, pub_points);


        // pcl::toROSMsg(*ptr_pc_accumulated, pub_points);
        pub_points.header.frame_id = "map";
        // test.header.frame_id = "map";
        result_pub.publish(pub_points);
      }
      ros::spinOnce();
    }

    return 0;
}
