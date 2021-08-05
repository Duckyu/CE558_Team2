#include "octomap.hpp"
bool N_octomap::ray_cast(n_cpp::RayCast::Request &req,
                         n_cpp::RayCast::Response &res){
  geometry_msgs::Point p0 = req.start;
  geometry_msgs::Point p1 = req.end;
  octomap::point3d start(p0.x, p0.y, p0.z);
  octomap::point3d direction(p1.x-p0.x, p1.y-p0.y, p1.z-p0.z);
  octomap::point3d end;
  double dist2first = pow(pow(p1.x-p0.x,2)+ pow(p1.y-p0.y,2)+ pow(p1.z-p0.z,2),0.5);
  #pragma omp parallel
  bool ray_result = m_octree->castRay(start, direction, end, false, dist2first);
  auto *end_unknown = m_octree->search(end,0);
  if (!ray_result && end_unknown)
  {
    res.visible = true;
  }
  else{
    res.visible = false;
  }
  return true;
}
bool N_octomap::eliminate_tri(n_cpp::EliminateTriple::Request &req,
                              n_cpp::EliminateTriple::Response &res){
  std::vector<geometry_msgs::Point> target = req.target;
  std::vector<geometry_msgs::Point>::iterator target_it;

  #pragma omp parallel for
  for(target_it=target.begin(); target_it!=target.end(); target_it++){
    triple_ban_list.push_back(m_octree->coordToKey(target_it->x, target_it->y, target_it->z));
  }
  res.result = true;
  return true;
}

bool N_octomap::get_triple_points(n_cpp::get_triple_points::Request &req,
                                  n_cpp::get_triple_points::Response &res){

  ROS_ERROR("get_triple_points!!!");

  std::vector<octomap::OcTreeKey> temp;
  // std::vector<octomap::OcTreeKey> temp_ban = triple_ban_list;

  //bool is_this_triplepoint(octomap::OcTree *octree, octomap::OcTreeKey tripoint_candidate)
  std::vector<octomap::OcTreeKey>::iterator existing_tripoints;
  std::vector<octomap::OcTreeKey>::iterator ban_it;
  
  #pragma omp parallel for
  for (existing_tripoints=triple_points.begin(); existing_tripoints!=triple_points.end(); existing_tripoints++) {
    
    bool banned = false;
    #pragma omp parallel for
    for(ban_it=triple_ban_list.begin(); (ban_it!=triple_ban_list.end()) && !banned; ban_it++)
    {
      if(*existing_tripoints == *ban_it){banned=true;triple_ban_list.erase(ban_it);}
    }
    if(!banned && is_this_triplepoint(m_octree, *existing_tripoints)){
      temp.push_back(*existing_tripoints);
    }
  }
  std::cout << "existing tripoints left: " << temp.size()<< std::endl;
  // ROS_ERROR("existing tripoints checked!!!");

  octomap::KeyBoolMap::const_iterator new_tripoints;
  #pragma omp parallel for
  for (new_tripoints=m_octree->changedKeysBegin(); new_tripoints!=m_octree->changedKeysEnd(); new_tripoints++) {
    // octomap::OcTreeNode* node = m_octree->search(new_tripoints->first); // it->first is now key, since KeyBoolMap is structure.
    octomap::OcTreeNode* node = m_octree->search(new_tripoints->first); // it->first is now key, since KeyBoolMap is structure.
    if (node){
      if(m_octree->isNodeOccupied(node)){ // ----> has value
        if(is_this_triplepoint(m_octree, new_tripoints->first)){
          temp.push_back(new_tripoints->first);
        }
      }
    }
  }

  m_octree->resetChangeDetection(); //important!!!!


  triple_points = temp;

  std::vector<octomap::OcTreeKey>::iterator updated_tripoints_key;
  pcl::PointCloud<pcl::PointXYZ> updated_tripoints;
  sensor_msgs::PointCloud2 sens_updated_tripoints;
  #pragma omp parallel for
  for (updated_tripoints_key=triple_points.begin(); updated_tripoints_key!=triple_points.end(); ++updated_tripoints_key){
    pcl::PointXYZ temp(m_octree->keyToCoord(*updated_tripoints_key).x(), m_octree->keyToCoord(*updated_tripoints_key).y(), m_octree->keyToCoord(*updated_tripoints_key).z());
    updated_tripoints.push_back(temp);
  }
  sens_updated_tripoints = cloud2msg(updated_tripoints, "map");
  
  if(req.pub_pc == true){
    cvt_octo2pubpc(m_octree);
    tri_points_pub.publish(sens_updated_tripoints);
  }
  res.tripoints = sens_updated_tripoints;
  res.curr_pose = curr_pose;

  return true;
}
////////////////////////////////////////////////////////////////

// bool N_octomap::candidate_check(n_cpp::collision_check::Request &req,
//                                 n_cpp::collision_check::Response &res){
//   std::vector<geometry_msgs::Point> points = req.ref_points;
//   std::vector<geometry_msgs::Vector3> normals = req.ref_normals;
//   nav_msgs::Path candi;
//   std::vector<geometry_msgs::Vector3> candi_normals;
  
//   if(points.size() != normals.size()){std::cout << "Something wrong with reference points" << std::endl;}
//   else{
//     for(int i=0; i<normals.size(); ++i){
//       geometry_msgs::Vector3 normalized_normals;
//       normalized_normals.x = normals[i].x;
//       normalized_normals.y = normals[i].y;
//       normalized_normals.z = normals[i].z;
//       // double ratio = pow(pow(normals[i].x,2)+pow(normals[i].y,2)+pow(normals[i].z,2),0.5);
//       // normalized_normals.x = normals[i].x/ratio;
//       // normalized_normals.y = normals[i].y/ratio;
//       // normalized_normals.z = normals[i].z/ratio;

//       int search_depth = 0;
//       double pigeon_house_max = 1.733 * octomap_resolution;

//       bool small;
//       auto *small_ptr = m_octree->search(points[i].x + pigeon_house_max*normalized_normals.x,
//                                          points[i].y + pigeon_house_max*normalized_normals.y,
//                                          points[i].z + pigeon_house_max*normalized_normals.z,
//                                          search_depth);

//       bool small_alt;
//       auto *small_alt_ptr = m_octree->search(points[i].x - pigeon_house_max*normalized_normals.x,
//                                              points[i].y - pigeon_house_max*normalized_normals.y,
//                                              points[i].z - pigeon_house_max*normalized_normals.z,
//                                              search_depth);
//       if (small_ptr){
//         std::cout<<"duck0"<<std::endl;
//         if (m_octree->isNodeOccupied(small_ptr)){small = false;std::cout<<"duck f"<<std::endl;}////occupied
//         else{small = true;std::cout<<"duck1"<<std::endl;}////free
//       }
//       else{small = true;std::cout<<"duck2"<<std::endl;}//// unknown

//       if (small_alt_ptr){
//         std::cout<<"--duck0"<<std::endl;
//         if (m_octree->isNodeOccupied(small_alt_ptr)){small_alt = false;std::cout<<"duck f"<<std::endl;}////occupied
//         else{small_alt = true;std::cout<<"--duck1"<<std::endl;}////free
//       }
//       else{small_alt = true;std::cout<<"--duck2"<<std::endl;}//// unknown
//       geometry_msgs::PoseStamped temp_candi;
//       bool big = small;
//       bool big_alt = small_alt;
//       if(small){
//         temp_candi.pose.position.x = points[i].x + req.offset_distance*normalized_normals.x;
//         temp_candi.pose.position.y = points[i].y + req.offset_distance*normalized_normals.y;
//         temp_candi.pose.position.z = points[i].z + req.offset_distance*normalized_normals.z;
//         if(temp_candi.pose.position.z<0){ 
//           big = false;
//           temp_candi.pose.position.x = points[i].x - req.offset_distance*normalized_normals.x;
//           temp_candi.pose.position.y = points[i].y - req.offset_distance*normalized_normals.y;
//           temp_candi.pose.position.z = points[i].z - req.offset_distance*normalized_normals.z;
//           if(temp_candi.pose.position.z<0) {big_alt = false;}
//         }
//       }
//       else if(small_alt){
//         temp_candi.pose.position.x = points[i].x - req.offset_distance*normalized_normals.x;
//         temp_candi.pose.position.y = points[i].y - req.offset_distance*normalized_normals.y;
//         temp_candi.pose.position.z = points[i].z - req.offset_distance*normalized_normals.z;
//         if(temp_candi.pose.position.z<0) {big_alt = false;}
//       }
//       if(big || big_alt){
//         auto *ptr = m_octree->search(temp_candi.pose.position.x,
//                                      temp_candi.pose.position.y,
//                                      temp_candi.pose.position.z);
//         if (ptr){
//           if(!m_octree->isNodeOccupied(ptr)){
//             tf::Quaternion quat;
//             quat.setRPY(0, 0, M_PI+atan2(normalized_normals.y, normalized_normals.x));
//             quat.normalize();
//             temp_candi.pose.orientation.x = quat.getX();
//             temp_candi.pose.orientation.y = quat.getY();
//             temp_candi.pose.orientation.z = quat.getZ();
//             temp_candi.pose.orientation.w = quat.getW();

//             std::cout << "registered" <<std::endl;
            
//             candi.poses.push_back(temp_candi);
//             candi_normals.push_back(normalized_normals);
//           }
//         }////free
//       }
//     }
//   }
//   res.candidate = candi;
//   res.candidate_normals = candi_normals;

//   return true;
// }