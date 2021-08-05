#include "octoMapgen.h"

using namespace imSLAM;
using namespace unavlib;

octoMapgen::octoMapgen()
{
  //  static ros::Subscriber sub1 = m_nh.subscribe<imSLAM::de_node>("deepExpress/octoMapgen/nodeIn",1,&octoMapgen::callback_node,this);
  static ros::Subscriber sub1 = m_nh.subscribe<im_node>("imSLAM/octoMapgen/nodeIn",10000,&octoMapgen::callback_node,this);
  m_pub_octoMap = m_nh.advertise<sensor_msgs::PointCloud2>("imSLAM/octoMapgen/debug/octoMap",100);
  static ros::Subscriber sub4 = m_nh.subscribe<std_msgs::Int32>("/imSLAM/octoMapgen/octomapReq",1000,&octoMapgen::callback_octoPub,this);

  getparam();

  m_octree = new octomap::OcTree(m_resolution);
  m_octree->setProbHit(0.6);
  m_octree->setProbMiss(0.4);
}

octoMapgen::~octoMapgen()
{

}

void octoMapgen::callback_node(const imSLAM::im_node::ConstPtr& msg)
{
  Eigen::Matrix4f odomResult = cvt::geoPose2eigen(msg->odom);
  Eigen::Matrix4f lidarCenter = cvt::geoPose2eigen(msg->odom)*m_tf_sensor2lidar*m_tf_robot2sensor;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZI>());
  cvt::cloudmsg2cloudptr(msg->lidar,cloudIn);

  *cloudIn = cvt::cloud2cloudcut<pcl::PointXYZI>(*cloudIn,pcl::PointXYZ(0,0,1),1,80);

  pcl::transformPointCloud(*cloudIn,*cloudIn,lidarCenter);


  pcl::UniformSampling<pcl::PointXYZI> sampler;
  sampler.setRadiusSearch(m_resolution);
  sampler.setInputCloud(cloudIn);
  sampler.filter(*cloudIn);

  //*cloudIn = datahandle3d::voxelize_NoOverflow(cloudIn, m_resolution/2.0);
//  m_pub_octoMap.publish(cvt::cloud2msg(*cloudIn));

  octomap::point3d sensorpt(lidarCenter(0,3),lidarCenter(1,3),lidarCenter(2,3));
  octomap::Pointcloud temp_octo_pointcloud;
  for(int i=0;i<cloudIn->size();i++)
  {
     temp_octo_pointcloud.push_back(cloudIn->at(i).x,cloudIn->at(i).y,cloudIn->at(i).z);
  }
  m_octree->insertPointCloudRays(temp_octo_pointcloud,sensorpt);

  static int idx = 0;
  std::cout<<"[OCTOMAP] NODE "<<idx++<<" INSERTED"<<std::endl;
}

void octoMapgen::callback_octoPub(const std_msgs::Int32::ConstPtr &msg)
{
  std::cout<<"[OCTOMAP] MAKE OCTOMAP!"<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr occupiedCloud(new pcl::PointCloud<pcl::PointXYZ>());
  for(octomap::OcTree::iterator it = m_octree->begin(); it!=m_octree->end(); ++it)
  {
    if(m_octree->isNodeOccupied(*it))
    {
      occupiedCloud->push_back(pcl::PointXYZ(it.getCoordinate().x(),it.getCoordinate().y(),
                                            it.getCoordinate().z()));
    }
//      else
//      {
//        freeCloud->push_back(pcl::PointXYZ(it.getCoordinate().x(),it.getCoordinate().y(),
//                                           it.getCoordinate().z()));
//      }
  }
  m_pub_octoMap.publish(cvt::cloud2msg(*occupiedCloud));
  std::cout<<"[OCTOMAP] OCTOMAP PUBLISHED"<<std::endl;
}

void octoMapgen::getparam()
{
  std::vector<double> tfIn;
  if(m_nh.getParam("tf/robot2lidarBase",tfIn))
  {
    if(tfIn.size() == 7)
    {
      geometry_msgs::Pose geoTF;
      geoTF.position.x = tfIn[0];
      geoTF.position.y = tfIn[1];
      geoTF.position.z = tfIn[2];
      geoTF.orientation.x = tfIn[3];
      geoTF.orientation.y = tfIn[4];
      geoTF.orientation.z = tfIn[5];
      geoTF.orientation.w = tfIn[6];
      m_tf_robot2sensor = cvt::geoPose2eigen(geoTF);
      std::cout<<"ROBOT 2 SENSOR : \n"<<geoTF<<std::endl;
    }
    else
    {
      ROS_ERROR("TF MUST BE X,Y,Z,Qx,Qy,Qw,Qz");
      exit(0);
    }
  }
  if(m_nh.getParam("tf/lidarBase2lidar",tfIn))
  {
    if(tfIn.size() == 7)
    {
      geometry_msgs::Pose geoTF;
      geoTF.position.x = tfIn[0];
      geoTF.position.y = tfIn[1];
      geoTF.position.z = tfIn[2];
      geoTF.orientation.x = tfIn[3];
      geoTF.orientation.y = tfIn[4];
      geoTF.orientation.z = tfIn[5];
      geoTF.orientation.w = tfIn[6];
      m_tf_sensor2lidar = cvt::geoPose2eigen(geoTF);
      std::cout<<"SENSOR 2 LIDAR : \n"<<geoTF<<std::endl;
    }
    else
    {
      ROS_ERROR("TF MUST BE X,Y,Z,Qx,Qy,Qw,Qz");
      exit(0);
    }
  }
  //  m_nh.param<bool>("sensorOn/odom",m_flag_odomOn,false);
    m_nh.param<float>("octomap/resolution",m_resolution,0.1);
}
