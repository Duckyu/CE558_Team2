#ifndef CONVT_H
#define CONVT_H

/** \cond */
#include <vector>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
/** \endcond */



namespace unavlib
{
  namespace cvt
  {
    Eigen::Matrix4f mat2eigen(cv::Mat mat);
    cv::Mat eigen2mat(Eigen::Matrix4f mat);
    void mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw);
    cv::Mat xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw);    
    Eigen::VectorXf eigen2xyzrpy(Eigen::Matrix4f mat);
    Eigen::Matrix4f xyzrpy2eigen(float x, float y, float z, float roll, float pitch, float yaw);
    Eigen::Matrix4f xyzrpy2eigen(Eigen::VectorXf xyzrpy);
    std_msgs::String str2msgStr(std::string str);
    std::string msgStr2str(std_msgs::String msgStr);
    geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose);
    Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose);
    Eigen::Matrix4d eigenf2eigend(Eigen::Matrix4f pose);
    Eigen::Matrix4f eigend2eigenf(Eigen::Matrix4d pose);
    cv::Point eigen2cvpt(Eigen::MatrixXf pt);
    pcl::PointXYZ eigen2pcl(Eigen::MatrixXf pt);
    Eigen::MatrixXf pcl2eigen(pcl::PointXYZ pt);

    Eigen::MatrixXf geoPoint2eigen(geometry_msgs::Point geoPoint);
    geometry_msgs::Point eigen2geoPoint(Eigen::MatrixXf point);
    void mat2sensorImg(cv::Mat mat, sensor_msgs::Image& sensorImg, std::string frameID = "map");
    cv::Mat sensorImg2mat(sensor_msgs::Image sensorImg);

    pcl::PointCloud<pcl::PointXYZ> laser2nonIcloud(sensor_msgs::LaserScan laser);
    pcl::PointCloud<pcl::PointXYZI> laser2cloud(sensor_msgs::LaserScan laser);
    sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud);
    sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZI> cloud);
    sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZRGB> cloud);
    pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg);
    sensor_msgs::PointCloud2 laser2cloudmsg(sensor_msgs::LaserScan laser);
    std::vector<cv::Point2d> laser2cloud_cvpts(sensor_msgs::LaserScan laser);

    template<typename T>
    pcl::PointCloud<T> cloud2cloudcut(pcl::PointCloud<T> cloud_in,pcl::PointXYZ cloud_center,float min_dist,float max_dist)
    {
        pcl::PointCloud<T> cloud_return;
        for(int i=0;i<cloud_in.size();i++)
        {
            T cloud_pt = cloud_in.at(i);
            float dist = sqrt(pow(cloud_pt.x - cloud_center.x,2)+
                              pow(cloud_pt.y - cloud_center.y,2)+
                              pow(cloud_pt.z - cloud_center.z,2));
            if((dist>min_dist) || (dist<max_dist)) cloud_return.push_back(cloud_pt);
        }

        return cloud_return;
    }

    nav_msgs::OccupancyGrid cvimg2occumap(cv::Mat cvimg, float resolution,cv::Point origin_cvpt);
    cv::Mat occumap2cvimg( nav_msgs::OccupancyGrid occumap);
    cv::Mat occumap2cvimg_unkblack( nav_msgs::OccupancyGrid occumap);
    void saveOccupanymap(std::string filepath, nav_msgs::OccupancyGrid gridmap_in);
    bool loadOccupancymap(std::string filepath, nav_msgs::OccupancyGrid& gridmap_out);
 }
}


#endif

