#include "mapper.hpp"

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

n_cpp::go_get_it move_cmd_send(){
  n_cpp::go_get_it srv;
  nav_msgs::Path local_path;

  ROS_INFO("path generation start!");
  

  ////Generate kd-tree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (ptr_pc_local);
  // ROS_INFO("normal estimation end!");

  //// Voxelization w\ big size leaf
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  double var_voxelsize = 1.0;
  voxel_filter.setInputCloud(ptr_pc_local);
  voxel_filter.setLeafSize(var_voxelsize, var_voxelsize, var_voxelsize);
  voxel_filter.filter(*cloud_dst);

  ROS_INFO("voxelization end!");

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud_dst->begin(); it!=cloud_dst->end(); ++it){
    pcl::PointXYZ searchPoint;
    searchPoint.x = it->x;
    searchPoint.y = it->y;
    searchPoint.z = it->z;

    // K nearest neighbor search
    int K = 10;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      // for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      //   std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
      //             << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
      //             << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
      //             << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;

      geometry_msgs::PoseStamped temp_pose;
      geometry_msgs::PoseStamped alt_temp_pose;
      double offset = 3.0;
      if(isnan(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[0]) == 0 &&
         isnan(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[1]) == 0 &&
         isnan(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[2]) == 0)
      {
        double normalization_rate = pow(pow(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[0],2) + 
                                        pow(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[1],2) +
                                        pow(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[2],2), 0.5);
        temp_pose.pose.position.x = ptr_pc_local->points[ pointIdxNKNSearch[0] ].x + offset * ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[0]/normalization_rate; 
        temp_pose.pose.position.y = ptr_pc_local->points[ pointIdxNKNSearch[0] ].y + offset * ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[1]/normalization_rate;
        temp_pose.pose.position.z = ptr_pc_local->points[ pointIdxNKNSearch[0] ].z + offset * ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[2]/normalization_rate;
        // std::cout << ptr_pc_local->points[ pointIdxNKNSearch[0] ].x << ", " 
        //           << ptr_pc_local->points[ pointIdxNKNSearch[0] ].y << ", " 
        //           << ptr_pc_local->points[ pointIdxNKNSearch[0] ].z <<std::endl;
        // std::cout << ptr_pc_local->points[ pointIdxNKNSearch[0] ].x << ", " 
        //           << ptr_pc_local->points[ pointIdxNKNSearch[0] ].y << ", " 
        //           << ptr_pc_local->points[ pointIdxNKNSearch[0] ].z <<std::endl;
        // std::cout << temp_pose.pose.position.x << ", " 
        //           << temp_pose.pose.position.y << ", " 
        //           << temp_pose.pose.position.z <<std::endl;
        tf::Quaternion q;
        // q.setRPY(std::atan(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[2] / ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[1]),
        //         std::atan(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[2] / ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[0]),
        //         std::atan(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[1] / ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[0]));
        q.setRPY(0, 0, std::atan(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[1] / ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[0]));
        temp_pose.pose.orientation.x = q.getX();
        temp_pose.pose.orientation.y = q.getY();
        temp_pose.pose.orientation.z = q.getZ();
        temp_pose.pose.orientation.w = q.getW();

        alt_temp_pose.pose.position.x = ptr_pc_local->points[ pointIdxNKNSearch[0] ].x - offset * ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[0]/normalization_rate; 
        alt_temp_pose.pose.position.y = ptr_pc_local->points[ pointIdxNKNSearch[0] ].y - offset * ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[1]/normalization_rate;
        alt_temp_pose.pose.position.z = ptr_pc_local->points[ pointIdxNKNSearch[0] ].z - offset * ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[2]/normalization_rate;
        // std::cout << ptr_pc_local->points[ pointIdxNKNSearch[0] ].x << ", " 
        //           << ptr_pc_local->points[ pointIdxNKNSearch[0] ].y << ", " 
        //           << ptr_pc_local->points[ pointIdxNKNSearch[0] ].z <<std::endl;
        // std::cout << ptr_pc_local->points[ pointIdxNKNSearch[0] ].x << ", " 
        //           << ptr_pc_local->points[ pointIdxNKNSearch[0] ].y << ", " 
        //           << ptr_pc_local->points[ pointIdxNKNSearch[0] ].z <<std::endl;
        // std::cout << temp_pose.pose.position.x << ", " 
        //           << temp_pose.pose.position.y << ", " 
        //           << temp_pose.pose.position.z <<std::endl;
        tf::Quaternion alt_q;
        // q.setRPY(std::atan(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[2] / ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[1]),
        //         std::atan(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[2] / ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[0]),
        //         std::atan(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[1] / ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[0]));
        alt_q.setRPY(0, 0, std::atan(ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[1] / ptr_pc_local_normal->points[pointIdxNKNSearch[0]].normal[0]));
        alt_temp_pose.pose.orientation.x = q.getX();
        alt_temp_pose.pose.orientation.y = q.getY();
        alt_temp_pose.pose.orientation.z = q.getZ();
        alt_temp_pose.pose.orientation.w = q.getW();

        int search_depth = 1;
        auto *ptr = m_octree->search(temp_pose.pose.position.x,temp_pose.pose.position.y,temp_pose.pose.position.z,search_depth);
        if (ptr){
          if (m_octree->isNodeOccupied(ptr)){
            ////occupied
            std::cout << "occupied" << std::endl;
            auto *alt_ptr = m_octree->search(alt_temp_pose.pose.position.x,alt_temp_pose.pose.position.y,alt_temp_pose.pose.position.z,search_depth);
            if (alt_ptr)
            {
              if (m_octree->isNodeOccupied(alt_ptr)){
                ////occupied
                std::cout << "alt occupied" << std::endl;
              }
              else{
                ////free
                std::cout << "alt free" << std::endl;
                if (isnan(alt_temp_pose.pose.orientation.x) == 0 &&
                    isnan(alt_temp_pose.pose.orientation.y) == 0 &&
                    isnan(alt_temp_pose.pose.orientation.z) == 0 &&
                    isnan(alt_temp_pose.pose.orientation.w) == 0){
                      if(alt_temp_pose.pose.position.z > 0) local_path.poses.push_back(alt_temp_pose);
                    }
              }
            }
            else
            {
              //// alt unknown
              std::cout << "alt unknown" << std::endl;
              if (isnan(alt_temp_pose.pose.orientation.x) == 0 &&
                  isnan(alt_temp_pose.pose.orientation.y) == 0 &&
                  isnan(alt_temp_pose.pose.orientation.z) == 0 &&
                  isnan(alt_temp_pose.pose.orientation.w) == 0){
                    if(alt_temp_pose.pose.position.z > 0) local_path.poses.push_back(alt_temp_pose);
                  }
            }
          }
          else{
            ////free
            std::cout << "free" << std::endl;
            if (isnan(temp_pose.pose.orientation.x) == 0 &&
                isnan(temp_pose.pose.orientation.y) == 0 &&
                isnan(temp_pose.pose.orientation.z) == 0 &&
                isnan(temp_pose.pose.orientation.w) == 0)
                {
                  if(temp_pose.pose.position.z > 0) local_path.poses.push_back(temp_pose);
                } 
          }
        }
        else{
          //// unknown
          std::cout << "unknown" << std::endl;
          if (isnan(temp_pose.pose.orientation.x) == 0 &&
              isnan(temp_pose.pose.orientation.y) == 0 &&
              isnan(temp_pose.pose.orientation.z) == 0 &&
              isnan(temp_pose.pose.orientation.w) == 0){
                if(temp_pose.pose.position.z > 0) local_path.poses.push_back(temp_pose);
              }
        }
      }
    }
    pointIdxNKNSearch.clear();
    pointNKNSquaredDistance.clear();
  }
  local_path.header.frame_id = "map";
  srv.request.path = local_path;
  ptr_pc_local->clear();
  ptr_pc_local_normal->clear();
  local_map = true;
  ROS_INFO("path generation end!");
  return srv;
}

bool path_require(n_cpp::path_require::Request  &req,
                  n_cpp::path_require::Response &res)
{
  geometry_msgs::Pose curr_UAV = posepose.pose;
  local_map = false;
  res.result = true;
  // move_cmd_send();
  call_move = true;
  return true;
}

void cb_points_up(const sensor_msgs::PointCloud2::ConstPtr& msg){
  static ros::Time last_request;
  // std::cout << "up last_request : " << last_request.toSec() << std::endl;
  ros::Duration diff = ros::Time::now() - last_request;
  // std::cout << "up diff : " << diff.toSec() << std::endl;
  
  if (diff > ros::Duration(0.2)){
    // std::cout << "UP!!!!" << std::endl;

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

    pcl::PassThrough<pcl::PointXYZ> tran_filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_pc (new pcl::PointCloud<pcl::PointXYZ>());
    tran_filter.setInputCloud(tran_pc);
    tran_filter.setFilterFieldName("z");
    tran_filter.setFilterLimits(0.1, 100.0);
    tran_filter.filter(*final_pc);

    octomap::Pointcloud temp_pcl;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = final_pc->begin(); it!=final_pc->end(); ++it){
      temp_pcl.push_back(it->x, it->y, it->z);
    }
    // m_octree->insertPointCloudRays(temp_pcl, octomap::point3d(map_t_up(0,3),map_t_up(1,3),map_t_up(2,3)));
    m_octree->insertPointCloud(temp_pcl, octomap::point3d(map_t_up(0,3),map_t_up(1,3),map_t_up(2,3)));
    // *ptr_pc_accumulated += *final_pc;
    // if (local_map) *ptr_pc_local += *final_pc;
    

    last_request = ros::Time::now();
    cloudIn->clear();
    cloudOut->clear();
    tran_pc->clear();
    final_pc->clear();
  }
}

void cb_points_front(const sensor_msgs::PointCloud2::ConstPtr& msg){
  static ros::Time last_request;
  // std::cout << "last_request : " << last_request.toSec() << std::endl;
  ros::Duration diff = ros::Time::now() - last_request;
  // std::cout << "diff : " << diff.toSec() << std::endl;
  
  if (diff > ros::Duration(0.2)){
    // std::cout << "FRONT!!!!" << std::endl;


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

    pcl::PassThrough<pcl::PointXYZ> tran_filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cut_pc (new pcl::PointCloud<pcl::PointXYZ>());
    tran_filter.setInputCloud(tran_pc);
    tran_filter.setFilterFieldName("z");
    tran_filter.setFilterLimits(0.1, 100.0);
    tran_filter.filter(*ground_cut_pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_pc (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    double var_voxelsize = 0.05;
    voxel_filter.setInputCloud(ground_cut_pc);
    voxel_filter.setLeafSize(var_voxelsize, var_voxelsize, var_voxelsize);
    voxel_filter.filter(*final_pc);

    octomap::Pointcloud temp_pcl;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = final_pc->begin(); it!=final_pc->end(); ++it){
      temp_pcl.push_back(it->x, it->y, it->z);
    } 
    // m_octree->insertPointCloudRays(temp_pcl, octomap::point3d(map_t_front(0,3),map_t_front(1,3),map_t_front(2,3)));
    m_octree->insertPointCloud(temp_pcl, octomap::point3d(map_t_front(0,3),map_t_front(1,3),map_t_front(2,3)));
    // *ptr_pc_accumulated += *final_pc;
    if (local_map){
      ////normal vector estimation w\ index
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      // pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
      ne.setInputCloud (ptr_pc_local->makeShared());
      ne.setSearchMethod (tree);
      ne.setRadiusSearch (0.03);
      ne.compute (*cloud_normals);

      *ptr_pc_local += *final_pc;
      *ptr_pc_local_normal += *cloud_normals;
      std::cout << "ptr_pc_local : " << ptr_pc_local-> size()<< std::endl;
    } 
    

    last_request = ros::Time::now();
    cloudIn->clear();
    cloudOut->clear();
    tran_pc->clear();
    ground_cut_pc->clear();
    final_pc->clear();
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
    ros::Publisher octo_result_pub = n.advertise<sensor_msgs::PointCloud2>("result_octo_occu", 1000);
    ros::Publisher octo_result_free_pub = n.advertise<sensor_msgs::PointCloud2>("result_octo_free", 1000);
    // ros::Publisher result_pub = n.advertise<sensor_msgs::PointCloud2>("result", 1000);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("result_path", 1000);
    ros::Publisher local_path_pub = n.advertise<nav_msgs::Path>("local_path", 1000);

    ros::Subscriber points_sub_front = n.subscribe("/camera_front/depth/points", 1000, cb_points_front);
    ros::Subscriber points_sub_up = n.subscribe("/camera_up/depth/points", 1000, cb_points_up);
    // ros::Subscriber sub_model = n.subscribe("/gazebo/model_states", 1000, cb_model_state);
    ros::Subscriber sub_tf = n.subscribe("/tf", 1000, cb_tf);
    ros::Subscriber sub_st_tf = n.subscribe("/tf_static", 1000, cb_st_tf);

    ros::ServiceClient move_cmd_client = n.serviceClient<n_cpp::go_get_it>("go_get_it");
    ros::ServiceServer pathgen_cmd_service = n.advertiseService("path_require", path_require);

    ros::Rate loop_rate(20);

    set_trans();
    setup_octomap();
    std::cout << "Let's getting to rumble!" << std::endl;

    while(ros::ok()){
      
      sensor_msgs::PointCloud2 pub_octo;
      sensor_msgs::PointCloud2 pub_octo_free;
      // sensor_msgs::PointCloud2 pub_points;
      // pcl::PointCloud<pcl::PointXYZ> cloud_dst;
      // pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      // double var_voxelsize = 0.05;
      // voxel_filter.setInputCloud(ptr_pc_accumulated);
      // voxel_filter.setLeafSize(var_voxelsize, var_voxelsize, var_voxelsize);
      // voxel_filter.filter(cloud_dst);
      // *ptr_pc_accumulated = cloud_dst;

      // pub_points.header.frame_id = "map";
      pcl::PointCloud<pcl::PointXYZ>::Ptr octo_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr free_octo_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
      for (octomap::OcTree::iterator it=m_octree->begin(); it!=m_octree->end(); ++it){
        if(m_octree->isNodeOccupied(*it))
        {
          octo_pcl_pub->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
        }
        else
        {
          free_octo_pcl_pub->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
        }
      }
      
      // pcl::toROSMsg<pcl::PointXYZ>(*ptr_pc_accumulated, pub_points);
      pcl::toROSMsg<pcl::PointXYZ>(*octo_pcl_pub, pub_octo);
      pcl::toROSMsg<pcl::PointXYZ>(*free_octo_pcl_pub, pub_octo_free);

      pathpath.header.frame_id = "map";
      // pub_points.header.frame_id = "map";
      pub_octo.header.frame_id = "map";
      pub_octo_free.header.frame_id = "map";

      octo_result_pub.publish(pub_octo);
      octo_result_free_pub.publish(pub_octo_free);
      // result_pub.publish(pub_points);
      path_pub.publish(pathpath);

      // ROS_INFO("call move?");
      if (call_move){
        n_cpp::go_get_it srv;
        srv = move_cmd_send();
        // move_cmd_client.call(srv);
        local_path_pub.publish(srv.request.path);
        local_map = true;
        call_move = false;
        ROS_INFO("Call move!");
      }
      // else{
      //   ROS_INFO("NOPE!");
      // }

      octo_pcl_pub->clear();
      // cloud_dst.clear();
      loop_rate.sleep();
      ros::spinOnce();
    }

    return 0;
}
