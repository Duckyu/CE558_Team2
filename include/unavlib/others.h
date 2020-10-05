#ifndef OTHERS_H
#define OTHERS_H

/** \cond */
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include "opencv2/opencv.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include "sensor_msgs/CameraInfo.h"
/** \endcond */


namespace unavlib
{
  namespace probabilistic
  {
    double GaussianRand();
  }

  namespace time
  {
    unsigned long GetTickCount(int option = 0);   //0 : ms / 1:ns
    void tic();
    void toc();
  }

  namespace xtion
  {
    void update_caminfo(sensor_msgs::CameraInfo caminfo);
    void update_range(double min = 0.35, double max = 0.5);
    pcl::PointCloud<pcl::PointXYZRGB> get3D(cv::Mat img, cv::Mat depth, int resize = 1);
    pcl::PointCloud<pcl::PointXYZ> get3D(std::vector<float> pts, std::vector<float> depths);
    Eigen::MatrixXf get3D_eigen(std::vector<float> pts, std::vector<float> depths);
  }

  namespace datahandle3d
  {
    cv::Vec3b heightcolor(double h);  // h : 0-1 -> color : blue - red
    pcl::PointCloud<pcl::PointXYZ> voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float size = 0.1);
    pcl::PointCloud<pcl::PointXYZRGB> voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float size = 0.1);
    pcl::PointCloud<pcl::PointXYZI> voxelize(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float size = 0.1);
  }


}


#endif

