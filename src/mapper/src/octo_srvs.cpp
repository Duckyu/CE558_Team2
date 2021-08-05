#include "octomap.hpp"

bool N_octomap::get_changed_octomap(n_cpp::get_octomap::Request &req,
                                    n_cpp::get_octomap::Response &res){
  if(req.terminate_flag){
    cvt_octo2pubpc(m_octree);
  }
  else{
    sensor_msgs::PointCloud2 octo_change;
    geometry_msgs::PoseStamped pose;

    pose = curr_pose;

    pcl::PointCloud<pcl::PointXYZ>::Ptr change_octo_pcl(new pcl::PointCloud<pcl::PointXYZ>());

    octomap::KeyBoolMap::const_iterator it_change;
    for (it_change=m_octree->changedKeysBegin(); it_change!=m_octree->changedKeysEnd(); it_change++) {
      octomap::OcTreeNode* node = m_octree->search(it_change->first); // it->first is now key, since KeyBoolMap is structure.
      if (node){
        if(m_octree->isNodeOccupied(node)){ // ----> has value
          change_octo_pcl->push_back(pcl::PointXYZ(m_octree->keyToCoord(it_change->first).x(), m_octree->keyToCoord(it_change->first).y(), m_octree->keyToCoord(it_change->first).z()));
        }
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr change_octo_sor(new pcl::PointCloud<pcl::PointXYZ>());

    // int num_neigbor_points = 10; moved to header
    // double std_multiplier = 1.0; moved to header

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (change_octo_pcl);
    sor.setMeanK (num_neigbor_points);
    sor.setStddevMulThresh (std_multiplier);
    sor.filter(*change_octo_sor);

    // octo_change = cloud2msg(*change_octo_pcl, "map");
    octo_change = cloud2msg(*change_octo_sor, "map");
    if(req.pub_pc == true){
      cvt_octo2pubpc(m_octree);
      octo_change_pub.publish(octo_change);
    }
    res.octo_change = octo_change;
    res.curr_pose = pose;

    m_octree->resetChangeDetection(); //important!!!!
  }
  return true;
}

bool N_octomap::candidate_check(n_cpp::collision_check::Request &req,
                                n_cpp::collision_check::Response &res){
  std::vector<geometry_msgs::Point> points = req.ref_points;
  std::vector<geometry_msgs::Vector3> normals = req.ref_normals;
  std::vector<geometry_msgs::Point> remain_ref_points;
  nav_msgs::Path candi;
  std::vector<geometry_msgs::Vector3> candi_normals;
  
  if(points.size() != normals.size()){std::cout << "Something wrong with reference points" << std::endl;}
  else{
    for(int i=0; i<normals.size(); ++i){
      geometry_msgs::Vector3 normalized_normals;
      if(normals[i].z > 0.8){
        std::cout << "looking to the sky"<<std::endl;
        double new_normal = pow(pow(normals[i].x+2.0*normals[i].x/pow(pow(normals[i].x,2)+pow(normals[i].y,2),0.5)*normals[i].z,2)+
                                pow(normals[i].y+2.0*normals[i].y/pow(pow(normals[i].x,2)+pow(normals[i].y,2),0.5)*normals[i].z,2)+
                                pow(normals[i].z,2),0.5);
        normalized_normals.x = (normals[i].x+2.0*normals[i].x/pow(pow(normals[i].x,2)+pow(normals[i].y,2),0.5)*normals[i].z)/new_normal;
        normalized_normals.y = (normals[i].y+2.0*normals[i].y/pow(pow(normals[i].x,2)+pow(normals[i].y,2),0.5)*normals[i].z)/new_normal;
        normalized_normals.z = normals[i].z/new_normal;
        std::cout << "normal:     (" <<normals[i].x <<", " <<normals[i].y <<", " <<normals[i].z <<")" <<std::endl;
        std::cout << "new normal: (" <<normalized_normals.x <<", " <<normalized_normals.y <<", " <<normalized_normals.z <<")" <<std::endl;
      }
      else{
        normalized_normals.x = normals[i].x;
        normalized_normals.y = normals[i].y;
        normalized_normals.z = normals[i].z;
      }

      geometry_msgs::PoseStamped temp_candi;

      octomap::point3d start_point(points[i].x,
                                   points[i].y,
                                   points[i].z);
      octomap::point3d direction(normalized_normals.x, normalized_normals.y, normalized_normals.z);
      octomap::point3d start_offset(0.9*octomap_resolution*direction.x(),
                                    0.9*octomap_resolution*direction.y(),
                                    0.9*octomap_resolution*direction.z());
      octomap::point3d end_point;
      bool ignoreUnknownCells = false;
      double maxRange = req.offset_distance;

      octomap::point3d test = start_point-start_offset;
      // std::cout << start_point.x() << start_offset.x() << test.x() << std::endl;

      #pragma omp parallel
      bool ray_result = m_octree->castRay(start_point+start_offset, direction, end_point, ignoreUnknownCells, maxRange-0.9*octomap_resolution);

      auto *end_unknown = m_octree->search(end_point,0);

      if (!ray_result && end_unknown)
      {
        tf::Quaternion quat;
        quat.setRPY(0, 0, M_PI+atan2(direction.y(), direction.x()));
        quat.normalize();

        temp_candi.pose.position.x = start_point.x() + req.offset_distance*direction.x();
        temp_candi.pose.position.y = start_point.y() + req.offset_distance*direction.y();
        temp_candi.pose.position.z = start_point.z() + req.offset_distance*direction.z();
        temp_candi.pose.orientation.x = quat.getX();
        temp_candi.pose.orientation.y = quat.getY();
        temp_candi.pose.orientation.z = quat.getZ();
        temp_candi.pose.orientation.w = quat.getW();

        std::cout << "registered" <<std::endl;

        remain_ref_points.push_back(points[i]);
        candi.poses.push_back(temp_candi);
        candi_normals.push_back(normalized_normals);
      }
      else
      {
        ROS_ERROR("normal crack");

        #pragma omp parallel
        bool alt_ray_result = m_octree->castRay(start_point-start_offset, -direction, end_point, ignoreUnknownCells, maxRange-0.9*octomap_resolution);

        auto *alt_end_unknown = m_octree->search(end_point,0);
      
        if (!alt_ray_result && alt_end_unknown)
        {
          tf::Quaternion quat;
          quat.setRPY(0, 0, M_PI+atan2(-direction.y(), -direction.x()));
          quat.normalize();

          temp_candi.pose.position.x = start_point.x() - req.offset_distance*direction.x();
          temp_candi.pose.position.y = start_point.y() - req.offset_distance*direction.y();
          temp_candi.pose.position.z = start_point.z() - req.offset_distance*direction.z();
          temp_candi.pose.orientation.x = quat.getX();
          temp_candi.pose.orientation.y = quat.getY();
          temp_candi.pose.orientation.z = quat.getZ();
          temp_candi.pose.orientation.w = quat.getW();

          std::cout << "registered" <<std::endl;

          remain_ref_points.push_back(points[i]);
          candi.poses.push_back(temp_candi);
          candi_normals.push_back(normalized_normals);
        }
        else{
          ROS_ERROR("alt normal crack");
        }
      }
    }
  }

  //     int search_depth = 0;
  //     double pigeon_house_max = 1.733 * octomap_resolution;

  //     bool small;
  //     auto *small_ptr = m_octree->search(points[i].x + pigeon_house_max*normalized_normals.x,
  //                                        points[i].y + pigeon_house_max*normalized_normals.y,
  //                                        points[i].z + pigeon_house_max*normalized_normals.z,
  //                                        search_depth);

  //     bool small_alt;
  //     auto *small_alt_ptr = m_octree->search(points[i].x - pigeon_house_max*normalized_normals.x,
  //                                            points[i].y - pigeon_house_max*normalized_normals.y,
  //                                            points[i].z - pigeon_house_max*normalized_normals.z,
  //                                            search_depth);
  //     if (small_ptr){
  //       std::cout<<"duck0"<<std::endl;
  //       if (m_octree->isNodeOccupied(small_ptr)){small = false;std::cout<<"duck f"<<std::endl;}////occupied
  //       else{small = true;std::cout<<"duck1"<<std::endl;}////free
  //     }
  //     else{small = true;std::cout<<"duck2"<<std::endl;}//// unknown

  //     if (small_alt_ptr){
  //       std::cout<<"--duck0"<<std::endl;
  //       if (m_octree->isNodeOccupied(small_alt_ptr)){small_alt = false;std::cout<<"duck f"<<std::endl;}////occupied
  //       else{small_alt = true;std::cout<<"--duck1"<<std::endl;}////free
  //     }
  //     else{small_alt = true;std::cout<<"--duck2"<<std::endl;}//// unknown
  //     geometry_msgs::PoseStamped temp_candi;
  //     bool big = small;
  //     bool big_alt = small_alt;
  //     if(small){
  //       temp_candi.pose.position.x = points[i].x + req.offset_distance*normalized_normals.x;
  //       temp_candi.pose.position.y = points[i].y + req.offset_distance*normalized_normals.y;
  //       temp_candi.pose.position.z = points[i].z + req.offset_distance*normalized_normals.z;
  //       if(temp_candi.pose.position.z<0){ 
  //         big = false;
  //         temp_candi.pose.position.x = points[i].x - req.offset_distance*normalized_normals.x;
  //         temp_candi.pose.position.y = points[i].y - req.offset_distance*normalized_normals.y;
  //         temp_candi.pose.position.z = points[i].z - req.offset_distance*normalized_normals.z;
  //         if(temp_candi.pose.position.z<0) {big_alt = false;}
  //       }
  //     }
  //     else if(small_alt){
  //       temp_candi.pose.position.x = points[i].x - req.offset_distance*normalized_normals.x;
  //       temp_candi.pose.position.y = points[i].y - req.offset_distance*normalized_normals.y;
  //       temp_candi.pose.position.z = points[i].z - req.offset_distance*normalized_normals.z;
  //       if(temp_candi.pose.position.z<0) {big_alt = false;}
  //     }
  //     if(big || big_alt){
  //       auto *ptr = m_octree->search(temp_candi.pose.position.x,
  //                                    temp_candi.pose.position.y,
  //                                    temp_candi.pose.position.z);
  //       if (ptr){
  //         if(!m_octree->isNodeOccupied(ptr)){
  //           tf::Quaternion quat;
  //           quat.setRPY(0, 0, M_PI+atan2(normalized_normals.y, normalized_normals.x));
  //           quat.normalize();
  //           temp_candi.pose.orientation.x = quat.getX();
  //           temp_candi.pose.orientation.y = quat.getY();
  //           temp_candi.pose.orientation.z = quat.getZ();
  //           temp_candi.pose.orientation.w = quat.getW();

  //           std::cout << "registered" <<std::endl;
            
  //           candi.poses.push_back(temp_candi);
  //           candi_normals.push_back(normalized_normals);
  //         }
  //       }////free
  //     }
  //   }
  // }
  res.remain_ref_points = remain_ref_points;
  res.candidate = candi;
  res.candidate_normals = candi_normals;
  if(candi_normals.size() < valid_min_num){res.validity = false;}
  else{res.validity = true;}

  return true;
}