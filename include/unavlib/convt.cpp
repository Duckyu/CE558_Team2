#include "convt.h"

namespace unavlib
{

  namespace cvt
  {
    Eigen::Matrix4f mat2eigen(cv::Mat mat)
    {
        Eigen::Matrix4f result =  Eigen::Matrix4f::Identity();

        result(0,0) = mat.at<float>(0,0);
        result(0,1) = mat.at<float>(0,1);
        result(0,2) = mat.at<float>(0,2);
        result(0,3) = mat.at<float>(0,3);

        result(1,0) = mat.at<float>(1,0);
        result(1,1) = mat.at<float>(1,1);
        result(1,2) = mat.at<float>(1,2);
        result(1,3) = mat.at<float>(1,3);

        result(2,0) = mat.at<float>(2,0);
        result(2,1) = mat.at<float>(2,1);
        result(2,2) = mat.at<float>(2,2);
        result(2,3) = mat.at<float>(2,3);

        result(3,0) = mat.at<float>(3,0);
        result(3,1) = mat.at<float>(3,1);
        result(3,2) = mat.at<float>(3,2);
        result(3,3) = mat.at<float>(3,3);

        return result;
    }

    cv::Mat eigen2mat(Eigen::Matrix4f mat)
     {
         cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);
         result.at<float>(0,0) = mat(0,0);
         result.at<float>(0,1) = mat(0,1);
         result.at<float>(0,2) = mat(0,2);
         result.at<float>(0,3) = mat(0,3);

         result.at<float>(1,0) = mat(1,0);
         result.at<float>(1,1) = mat(1,1);
         result.at<float>(1,2) = mat(1,2);
         result.at<float>(1,3) = mat(1,3);

         result.at<float>(2,0) = mat(2,0);
         result.at<float>(2,1) = mat(2,1);
         result.at<float>(2,2) = mat(2,2);
         result.at<float>(2,3) = mat(2,3);

         result.at<float>(3,0) = mat(3,0);
         result.at<float>(3,1) = mat(3,1);
         result.at<float>(3,2) = mat(3,2);
         result.at<float>(3,3) = mat(3,3);

         return result;
     }

    cv::Mat xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw)
    {
          cv::Mat rot_vec = cv::Mat::zeros(3,1,CV_32FC1);
          rot_vec.at<float>(0) = roll;
          rot_vec.at<float>(1) = pitch;
          rot_vec.at<float>(2) = yaw;

          cv::Mat rot_mat;
          cv::Rodrigues(rot_vec,rot_mat);

          cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);

          rot_mat.copyTo(result(cv::Rect(0,0,3,3)));

          result.at<float>(0,3) = x;
          result.at<float>(1,3) = y;
          result.at<float>(2,3) = z;

          result.at<float>(3,3) = 1;

          return result;
    }

    void mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw)
    {
        *x = mat.at<float>(0,3);
        *y = mat.at<float>(1,3);
        *z = mat.at<float>(2,3);

        cv::Mat rot_mat = cv::Mat(mat(cv::Rect(0,0,3,3)));

        cv::Mat rot_vec;
        cv::Rodrigues(rot_mat,rot_vec);
        *roll = rot_vec.at<float>(0);
        *pitch = rot_vec.at<float>(1);
        *yaw = rot_vec.at<float>(2);
    }




    Eigen::VectorXf eigen2xyzrpy(Eigen::Matrix4f mat)
    {
        Eigen::VectorXf result(6);
        mat2xyzrpy(eigen2mat(mat), &result[0], &result[1], &result[2], &result[3], &result[4], &result[5]);
        return result;
    }


    Eigen::Matrix4f xyzrpy2eigen(float x, float y, float z, float roll, float pitch, float yaw)
    {
        Eigen::Matrix4f result =  mat2eigen(xyzrpy2mat(x,y,z,roll,pitch,yaw));
        return result;
    }

    std_msgs::String str2msgStr(std::string str)
    {
      std_msgs::String msgStr;
      msgStr.data = str;
      return msgStr;
    }

    std::string msgStr2str(std_msgs::String msgStr)
    {
      return msgStr.data;
    }

    geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose)
    {
        geometry_msgs::Pose geoPose;


        tf::Matrix3x3 m;
        m.setValue((double)pose(0,0),
                (double)pose(0,1),
                (double)pose(0,2),
                (double)pose(1,0),
                (double)pose(1,1),
                (double)pose(1,2),
                (double)pose(2,0),
                (double)pose(2,1),
                (double)pose(2,2));

        tf::Quaternion q;
        m.getRotation(q);
        geoPose.orientation.x = q.getX();
        geoPose.orientation.y = q.getY();
        geoPose.orientation.z = q.getZ();
        geoPose.orientation.w = q.getW();

        geoPose.position.x = pose(0,3);
        geoPose.position.y = pose(1,3);
        geoPose.position.z = pose(2,3);

        return geoPose;
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

    Eigen::Matrix4d eigenf2eigend(Eigen::Matrix4f pose)
    {
      Eigen::Matrix4d result;
      for(int y = 0; y < 4; y++)
      {
        for(int x = 0; x < 4; x ++)
        {
          result(y,x) = pose(y,x);
        }
      }
      return result;
    }

    Eigen::Matrix4f eigend2eigenf(Eigen::Matrix4d pose)
    {
      Eigen::Matrix4f result;
      for(int y = 0; y < 4; y++)
      {
        for(int x = 0; x < 4; x ++)
        {
          result(y,x) = pose(y,x);
        }
      }
      return result;
    }

    cv::Point eigen2cvpt(Eigen::MatrixXf pt)
    {
      return cv::Point(pt(0),pt(1));
    }

    Eigen::MatrixXf geoPoint2eigen(geometry_msgs::Point geoPoint)
    {
      Eigen::MatrixXf result(4,1);
      result << geoPoint.x, geoPoint.y, geoPoint.z, 1;
      return result;
    }

    geometry_msgs::Point eigen2geoPoint(Eigen::MatrixXf point)
    {
      geometry_msgs::Point result;
      result.x = point(0);
      result.y = point(1);
      result.z = point(2);
      return result;
    }

    pcl::PointXYZ eigen2pcl(Eigen::MatrixXf pt)
    {
      pcl::PointXYZ result_xyz;
      result_xyz.x = pt(0);
      result_xyz.y = pt(1);
      result_xyz.z = pt(2);
      return result_xyz;
    }

    Eigen::MatrixXf pcl2eigen(pcl::PointXYZ pt)
    {
      Eigen::Matrix4f result(4,1);
      result << pt.x, pt.y, pt.z, 1;
      return result;
    }

    void mat2sensorImg(cv::Mat mat, sensor_msgs::Image& sensorImg, std::string frameID)
    {
        cv_bridge::CvImage bridge;
        mat.copyTo(bridge.image);
        bridge.header.frame_id = frameID;
        bridge.header.stamp = ros::Time::now();
        if(mat.type() == CV_8UC1)
        {
            bridge.encoding = sensor_msgs::image_encodings::MONO8;
        }
        else if(mat.type() == CV_8UC3)
        {
            bridge.encoding = sensor_msgs::image_encodings::BGR8;
        }
        else if(mat.type() == CV_32FC1)
        {
            bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        }
        else if(mat.type() == CV_16UC1)
        {
            bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        }
        else
        {
            std::cout <<"Error : mat type" << std::endl;

        }

        bridge.toImageMsg(sensorImg);
    }

    cv::Mat sensorImg2mat(sensor_msgs::Image sensorImg)
    {
        static cv_bridge::CvImagePtr cv_ptr;
        cv::Mat mat;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(sensorImg, sensorImg.encoding);
            mat = cv_ptr->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        return mat;
    }

    pcl::PointCloud<pcl::PointXYZI> laser2cloud(sensor_msgs::LaserScan laser)
    {
      static laser_geometry::LaserProjection projector;
      sensor_msgs::PointCloud2 cloud_ROS;
      projector.projectLaser(laser, cloud_ROS,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::fromROSMsg(cloud_ROS, cloud);

      return cloud;
    }
    pcl::PointCloud<pcl::PointXYZ> laser2nonIcloud(sensor_msgs::LaserScan laser)
    {
      static laser_geometry::LaserProjection projector;
      sensor_msgs::PointCloud2 cloud_ROS;
      projector.projectLaser(laser, cloud_ROS,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(cloud_ROS, cloud);

      return cloud;
    }

    sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZI> cloud)
    {
      sensor_msgs::PointCloud2 cloud_ROS;
      pcl::toROSMsg(cloud, cloud_ROS);
      cloud_ROS.header.frame_id = "map";
      return cloud_ROS;
    }
    sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud)
    {
      sensor_msgs::PointCloud2 cloud_ROS;
      pcl::toROSMsg(cloud, cloud_ROS);
      cloud_ROS.header.frame_id = "map";
      return cloud_ROS;
    }
    sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZRGB> cloud)
    {
      sensor_msgs::PointCloud2 cloud_ROS;
      pcl::toROSMsg(cloud, cloud_ROS);
      cloud_ROS.header.frame_id = "map";
      return cloud_ROS;
    }

    pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
    {
      pcl::PointCloud<pcl::PointXYZ> cloudresult;
      pcl::fromROSMsg(cloudmsg,cloudresult);
      return cloudresult;
    }

    sensor_msgs::PointCloud2 laser2cloudmsg(sensor_msgs::LaserScan laser)
    {
      static laser_geometry::LaserProjection projector;
      sensor_msgs::PointCloud2 cloud_ROS;
      projector.projectLaser(laser, cloud_ROS,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
      cloud_ROS.header.frame_id = "map";

      return cloud_ROS;
    }

    std::vector<cv::Point2d> laser2cloud_cvpts(sensor_msgs::LaserScan laser)
    {
      std::vector<cv::Point2d> result_pts;
      for(int i = 0; i < laser.ranges.size(); i++)
      {
        float dist = laser.ranges[i];
        float angle = laser.angle_min + laser.angle_increment * i;
        if(dist < laser.range_min || dist > laser.range_max)
          continue;
        cv::Point2d tmp_pt;
        tmp_pt.y = sin(angle) * dist;
        tmp_pt.x = cos(angle) * dist;
        result_pts.push_back(tmp_pt);

      }
      return result_pts;
    }

    nav_msgs::OccupancyGrid cvimg2occumap(cv::Mat cvimg, float resolution,cv::Point origin_cvpt)
    {
      nav_msgs::OccupancyGrid m_gridmap;
      m_gridmap.info.resolution = resolution;
      geometry_msgs::Pose origin;
      origin.position.x = -origin_cvpt.x * resolution; origin.position.y = -origin_cvpt.y * resolution;
      origin.orientation.w = 1;
      m_gridmap.info.origin = origin;
      m_gridmap.info.width = cvimg.size().width;
      m_gridmap.info.height = cvimg.size().height;
      //ROS_INFO("[CV 2 OCCUGRID CONVERSION] PUT CV DATA");
      for(int i=0;i<m_gridmap.info.width*m_gridmap.info.height;i++) m_gridmap.data.push_back(-1);
      //ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID SIZE : %d",m_gridmap.data.size());
      //ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);

      for(int y = 0; y < cvimg.size().height; y++)
      {
        for(int x = 0; x < cvimg.size().width; x++)
        {
          int tmpdata = cvimg.at<unsigned char>(y,x);
          int ttmpdata = -1; //Unknown
          if(tmpdata >= 150) //free
          {
            ttmpdata = (tmpdata - 250) / -2;
            if(ttmpdata < 0)
              ttmpdata = 0;
          }
          else if(tmpdata <= 98)
          {
            ttmpdata = (tmpdata - 200) / -2;
            if(ttmpdata > 100)
              ttmpdata = 100;
          }
          m_gridmap.data.at(x + m_gridmap.info.width * y) = ttmpdata;
        }
      }
      return m_gridmap;
    }

    cv::Mat occumap2cvimg( nav_msgs::OccupancyGrid occumap)
    {
      // unkown = -1 -> 99~149, free : 0:50 ->250:150, occupied :51:100 -> 0:98
      double resolution = occumap.info.resolution;
      cv::Point origin_cvpt(-occumap.info.origin.position.x / resolution,
                            -occumap.info.origin.position.y / resolution);
      cv::Size img_size;
      cv::Mat cvimg = cv::Mat::zeros( occumap.info.height,occumap.info.width,CV_8UC1);
      //ROS_INFO("[OCCUGRID 2 CVT CONVERSION] GRID SIZE : %d",occumap.data.size());
      //ROS_INFO("[OCCUGRID 2 CVT CONVERSION] CV ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);
      for(int pt = 0;pt < occumap.data.size();pt++)
      {
        int pt_y = pt / occumap.info.width;
        int pt_x = pt % occumap.info.width;
        int value = occumap.data.at(pt);
        unsigned char img_value;
        if(value == -1) img_value = 120;
        else if (value <= 50) img_value = 250 - 2 * value;
        else if (value >=51) img_value = 200 - 2 * value;
        cvimg.at<unsigned char>(pt_y,pt_x) = img_value;
      }
      return cvimg;
    }
    cv::Mat occumap2cvimg_unkblack( nav_msgs::OccupancyGrid occumap)
        {
          // unkown = -1 -> 99~149, free : 0:50 ->250:150, occupied :51:100 -> 0:98
          double resolution = occumap.info.resolution;
          cv::Mat cvimg = cv::Mat::zeros( occumap.info.height,occumap.info.width,CV_8UC1);
          for(int pt = 0;pt < occumap.data.size();pt++)
          {
            int pt_y = pt / occumap.info.width;
            int pt_x = pt % occumap.info.width;
            int value = occumap.data.at(pt);
            unsigned char img_value;
            if(value == -1) img_value = 250;
            else if (value <= 50) img_value = 250 - 2 * value;
            else if (value >=51) img_value = 200 - 2 * value;
            cvimg.at<unsigned char>(pt_y,pt_x) = img_value;
          }
          return cvimg;
        }

    void saveOccupanymap(std::string filepath, nav_msgs::OccupancyGrid gridmap_in)
    {
      cv::Mat occumat = occumap2cvimg(gridmap_in);
      std::stringstream strm_png;
      strm_png << filepath << ".png";
      std::stringstream strm_info;
      strm_info << filepath << ".csv";

      cv::imwrite(strm_png.str(),occumat);

      std::ofstream filesave(strm_info.str().c_str());
      if(filesave.is_open())
      {
        filesave << gridmap_in.info.resolution << "\n";
        filesave << gridmap_in.info.origin.position.x << "\n";
        filesave << gridmap_in.info.origin.position.y << "\n";
      }
      filesave.close();
    }

    bool loadOccupancymap(std::string filepath, nav_msgs::OccupancyGrid& gridmap_out)
    {
      std::stringstream strm_png;
      strm_png << filepath <<".png";
      std::stringstream strm_info;
      strm_info << filepath << ".csv";
      cv::Mat occumat = cv::imread(strm_png.str(),cv::IMREAD_GRAYSCALE);
      std::ifstream fileload(strm_info.str().c_str());
      float resolution,origin_x,origin_y;
      std::vector<std::string>   result;
      std::string line;
      if(!fileload.is_open())
      {
        std::cout << "Warning : Canot open occupancy map" << std::endl;
        return false;
      }
      while(std::getline(fileload,line)) result.push_back(line);
      if(result.size()!=3)
      {
        std::cout << "Warning : Canot open occupancy map (arguments)" << std::endl;
        return false;
      }
      resolution = std::atof(result.at(0).c_str());
      origin_x = std::atof(result.at(1).c_str());
      origin_y = std::atof(result.at(2).c_str());
      gridmap_out = cvimg2occumap(occumat, resolution,cv::Point(- origin_x / resolution,-origin_y / resolution));
      gridmap_out.header.frame_id = "map";

      return true;
    }
  }
}

