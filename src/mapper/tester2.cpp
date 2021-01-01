#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <thread>
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

// #include <n_cpp/points_node.h>
// #include <n_cpp/points_nodes.h>
// #include <n_cpp/path_terminate.h>
// #include <n_cpp/local_path.h>
// #include <n_cpp/global_path.h>

#include <bits/stdc++.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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

//#include <unavlib/convt.h>
//#include <unavlib/others.h>
sensor_msgs::PointCloud2 front_;
geometry_msgs::PoseStamped pose_previous, pose_current;

pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_pc_accumulated (new pcl::PointCloud<pcl::PointXYZ>());
pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>());



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

// void cb_points_front(const sensor_msgs::PointCloud2::ConstPtr& msg){
//   front_ = *msg;
//   pcl::fromROSMsg(*msg, *cloudIn);
//   // *ptr_pc_accumulated += *cloudIn;
//   // std::cout << "SIZE: "<< ptr_pc_accumulated->points.size() << std::endl;
// }

// void cb_model_state(const geometry_msgs::PoseStamped::ConstPtr& msg){
//   pose_previous = pose_current;
//   pose_current = *msg;
//   double points_time = front_.header.stamp.toSec();
//   double prev_pose_time = pose_previous.header.stamp.toSec();
//   double curr_pose_time = pose_current.header.stamp.toSec();

//   std::cout << "points_time : " << points_time << std::endl;
//   std::cout << "prev_pose_time : " << prev_pose_time << std::endl;
//   std::cout << "curr_pose_time : " << curr_pose_time << std::endl;    

//   if(points_time > prev_pose_time && points_time < curr_pose_time){
//     Eigen::Matrix4f trans;
//     trans = interpolation(pose_previous, pose_current, points_time);
//     // trans = geoPose2eigen();
//     pcl::PointCloud<pcl::PointXYZ>::Ptr tran_pc (new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::transformPointCloud(*cloudIn, *tran_pc, trans);
//     *ptr_pc_accumulated += *tran_pc;

//     std::cout << "register!" << std::endl;
//   }
// }

void callback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg,
              const geometry_msgs::PoseStamped::ConstPtr& pose_msg){
  pcl::PointCloud<pcl::PointXYZ>::Ptr tran_pc (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*pc_msg, *cloudIn);
  Eigen::Matrix4f trans;
  trans = geoPose2eigen(pose_msg->pose);
  pcl::transformPointCloud(*cloudIn, *tran_pc, trans);
  // *ptr_pc_accumulated += *tran_pc;
  *ptr_pc_accumulated += *cloudIn;
  std::cout << "register!" << std::endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tester");
    ros::NodeHandle n;
    ros::Publisher result_pub = n.advertise<sensor_msgs::PointCloud2>("result", 1000);

    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(n, "/camera_front/depth/points", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(n, "/UAV", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> PcSyncPolicy;
    message_filters::Synchronizer<PcSyncPolicy> sync(PcSyncPolicy(10), pc_sub, pose_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // ros::Subscriber points_sub_front = n.subscribe("/camera_front/depth/points", 1000, cb_points_front);
    // ros::Subscriber sub_UAV = n.subscribe("/UAV", 1000, cb_model_state);
    ros::Rate loop_rate(1);
    ros::Time registerTime = ros::Time::now();
    while(ros::ok()){
      // pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_dst(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::fromROSMsg<pcl::PointXYZ>(*front_, *ptr_cloud_dst);
      // *ptr_pc_accumulated = *ptr_cloud_dst;
      sensor_msgs::PointCloud2 pub_points;
      double var_voxelsize = 0.05;
      voxel_filter.setInputCloud(ptr_pc_accumulated);
      voxel_filter.setLeafSize(var_voxelsize, var_voxelsize, var_voxelsize);
      voxel_filter.filter(*ptr_pc_accumulated);
      pcl::toROSMsg<pcl::PointXYZ>(*ptr_pc_accumulated, pub_points);
      pub_points.header.frame_id = "map";

      // pub_points = *front_;
      // pub_points.header.frame_id = "map";
      std::cout << "SIZE: "<< ptr_pc_accumulated->points.size() << std::endl;
      result_pub.publish(pub_points);
      
      loop_rate.sleep();
      ros::spinOnce();
    }

    return 0;
}
