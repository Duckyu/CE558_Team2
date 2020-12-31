
#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <sstream>

#include <tf/transform_listener.h>

#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

nav_msgs::Path pathpath;
geometry_msgs::PoseStamped posepose;

Eigen::Matrix4f map_t_body, body_t_front, body_t_up;

pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_pc_accumulated (new pcl::PointCloud<pcl::PointXYZ>());

Eigen::Matrix4f front_static_trans = Eigen::Matrix4f::Identity();

void normalVectorEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
  *cloud = *pc_ptr;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  return *cloud_normals;
}

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

void cb_points_up(const sensor_msgs::PointCloud2::ConstPtr& msg){
  static ros::Time last_request;
  std::cout << "up last_request : " << last_request.toSec() << std::endl;
  ros::Duration diff = ros::Time::now() - last_request;
  std::cout << "up diff : " << diff.toSec() << std::endl;
  
  if (diff > ros::Duration(0.2)){
    std::cout << "UP!!!!" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr tran_pc (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloudIn);
    Eigen::Matrix4f map_t_up = map_t_body * body_t_up ;
    pcl::PassThrough<pcl::PointXYZ> full_filter;
    full_filter.setInputCloud(cloudIn);
    full_filter.setFilterFieldName("z");
    full_filter.setFilterLimits(0.8, 12.0);
    full_filter.filter(*cloudOut);
    posepose.header.frame_id = "map";
    pathpath.poses.push_back(posepose);

    pcl::transformPointCloud(*cloudOut, *tran_pc, map_t_up);

    octomap::Pointcloud temp_pcl;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = tran_pc->begin(); it!=tran_pc->end(); ++it){
      temp_pcl.push_back(it->x, it->y, it->z);
    }
    m_octree->insertPointCloudRays(temp_pcl, octomap::point3d(map_t_up(0,3),map_t_up(1,3),map_t_up(2,3)));
    // m_octree->insertPointCloud(temp_pcl, octomap::point3d(map_t_up(0,3),map_t_up(1,3),map_t_up(2,3)));

    *ptr_pc_accumulated += *tran_pc;

    last_request = ros::Time::now();
    cloudIn->clear();
    cloudOut->clear();
    tran_pc->clear();
  }
}

void cb_points_front(const sensor_msgs::PointCloud2::ConstPtr& msg){
  static ros::Time last_request;
  std::cout << "last_request : " << last_request.toSec() << std::endl;
  ros::Duration diff = ros::Time::now() - last_request;
  std::cout << "diff : " << diff.toSec() << std::endl;
  
  if (diff > ros::Duration(0.2)){
    std::cout << "FRONT!!!!" << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr tran_pc (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloudIn);
    Eigen::Matrix4f map_t_front = map_t_body * body_t_front ;
    pcl::PassThrough<pcl::PointXYZ> full_filter;
    full_filter.setInputCloud(cloudIn);
    full_filter.setFilterFieldName("z");
    full_filter.setFilterLimits(0.8, 12.0);
    full_filter.filter(*cloudOut);
    posepose.header.frame_id = "map";
    pathpath.poses.push_back(posepose);

    pcl::transformPointCloud(*cloudOut, *tran_pc, map_t_front);

    octomap::Pointcloud temp_pcl;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = tran_pc->begin(); it!=tran_pc->end(); ++it){
      temp_pcl.push_back(it->x, it->y, it->z);
    }
    m_octree->insertPointCloudRays(temp_pcl, octomap::point3d(map_t_front(0,3),map_t_front(1,3),map_t_front(2,3)));
    // m_octree->insertPointCloud(temp_pcl, octomap::point3d(map_t_front(0,3),map_t_front(1,3),map_t_front(2,3)));
    *ptr_pc_accumulated += *tran_pc;

    last_request = ros::Time::now();
    cloudIn->clear();
    cloudOut->clear();
    tran_pc->clear();
  }
}

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
    }
  }
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "tester");
    ros::NodeHandle n;
    ros::Publisher octo_result_pub = n.advertise<sensor_msgs::PointCloud2>("result_octo", 1000);
    ros::Publisher result_pub = n.advertise<sensor_msgs::PointCloud2>("result", 1000);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("result_path", 1000);

    ros::Subscriber points_sub_front = n.subscribe("/camera_front/depth/points", 1000, cb_points_front);
    ros::Subscriber points_sub_up = n.subscribe("/camera_up/depth/points", 1000, cb_points_up);
    // ros::Subscriber sub_model = n.subscribe("/gazebo/model_states", 1000, cb_model_state);
    ros::Subscriber sub_tf = n.subscribe("/tf", 1000, cb_tf);
    ros::Subscriber sub_st_tf = n.subscribe("/tf_static", 1000, cb_st_tf);
    

    ros::Rate loop_rate(20);

    set_trans();
    setup_octomap();
    std::cout << "Let's getting to rumble!" << std::endl;

    while(ros::ok()){
      
      sensor_msgs::PointCloud2 pub_octo;
      sensor_msgs::PointCloud2 pub_points;
      pcl::PointCloud<pcl::PointXYZ> cloud_dst;
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      double var_voxelsize = 0.05;
      voxel_filter.setInputCloud(ptr_pc_accumulated);
      voxel_filter.setLeafSize(var_voxelsize, var_voxelsize, var_voxelsize);
      voxel_filter.filter(cloud_dst);
      pcl::toROSMsg<pcl::PointXYZ>(cloud_dst, pub_points);
      *ptr_pc_accumulated = cloud_dst;

      // pub_points.header.frame_id = "map";
      pcl::PointCloud<pcl::PointXYZ>::Ptr octo_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
      for (octomap::OcTree::iterator it=m_octree->begin(); it!=m_octree->end(); ++it){
        if(m_octree->isNodeOccupied(*it))
        {
          octo_pcl_pub->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
        }
      }
      pcl::toROSMsg<pcl::PointXYZ>(*octo_pcl_pub, pub_points);

      pathpath.header.frame_id = "map";

      octo_result_pub.publish(pub_octo);
      result_pub.publish(pub_points);
      path_pub.publish(pathpath);
      octo_pcl_pub->clear();
      cloud_dst.clear();
      loop_rate.sleep();
      ros::spinOnce();
    }

    return 0;
}
