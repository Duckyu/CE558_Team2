#include "path_gen.hpp"

geometry_msgs::PoseStamped N_path::pose_correction_path_gen(sensor_msgs::PointCloud2 key_cloud)
{
	
}

n_cpp::NextStep::Response N_path::new_path_gen(bool escape)
{
	////Generate kd-tree
	// pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	nanoflann::KdTreeFLANN<pcl::PointXYZ> kdtree;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_octo_tri(new pcl::PointCloud<pcl::PointXYZ> ());
	// *pcl_octo_new = cloudmsg2cloud(octo_new);
	*pcl_octo_tri = cloudmsg2cloud(tripoints);
	std::cout << "tripoints size : " << pcl_octo_tri->size() << std::endl;
	n_cpp::collision_check candi;

	if(pcl_octo_tri->size()>0){
		// pcl::PointCloud<pcl::PointXYZ>::Ptr octo_tri_no_ground(new pcl::PointCloud<pcl::PointXYZ> ());
		// pcl::PassThrough<pcl::PointXYZ> full_filter;
		// full_filter.setInputCloud(pcl_octo_tri);
		// full_filter.setFilterFieldName("z");
		// full_filter.setFilterLimits(ground_thresh, 25.0);
		// full_filter.filter(*octo_tri_no_ground);

		// pcl::PointCloud<pcl::PointXYZ>::Ptr octo_new_down(new pcl::PointCloud<pcl::PointXYZ> ());
		// pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
		// voxel_filter.setInputCloud(octo_new_no_ground);
		// voxel_filter.setLeafSize(down_size, down_size, down_size);
		// voxel_filter.filter(*octo_new_down);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	    // tree->setInputCloud (octo_tri_no_ground);
	    tree->setInputCloud (pcl_octo_tri);

	    std::vector<pcl::PointIndices> cluster_indices;
	    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	    ec.setClusterTolerance (dist_ratio*octomap_resolution);
	    ec.setMinClusterSize (min_cluster);				// 한 군집의 최소 포인트 개수
	    ec.setMaxClusterSize (max_cluster);				// 한 군집의 최대 포인트 개수
	    ec.setSearchMethod (tree);				// 검색 방법 : tree 
	    ec.setInputCloud (pcl_octo_tri);	// cloud_filtered_v2에 클러스터링 결과를 입력
	    ec.extract (cluster_indices);
	    
	    // std::cout << "cluster_indices1" << cluster_indices.size() << std::endl;
	    if (cluster_indices.size() == 0){
	    	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		    ec.setClusterTolerance (2*dist_ratio*octomap_resolution);
		    ec.setMinClusterSize (min_cluster);				// 한 군집의 최소 포인트 개수
		    ec.setMaxClusterSize (max_cluster);				// 한 군집의 최대 포인트 개수
		    ec.setSearchMethod (tree);				// 검색 방법 : tree 
		    ec.setInputCloud (pcl_octo_tri);	// cloud_filtered_v2에 클러스터링 결과를 입력
		    ec.extract (cluster_indices);
	    	// std::cout << "cluster_indices2" << cluster_indices.size() << std::endl;
	    }

	    int closest_idx = 0;
	    int second_idx = 0;
	    int id = 0;
	    float compare_centroid_dist = -1.0;
	    pcl::PointIndices::Ptr closest_indices(new pcl::PointIndices ());
	    pcl::PointIndices::Ptr second_indices(new pcl::PointIndices ());
	    #pragma omp parallel for
	    for(std::vector<pcl::PointIndices>::iterator iter = cluster_indices.begin(); iter != cluster_indices.end(); iter++, id++)
	    { 
    		Eigen::Vector4f temp_centroid;
    		// pcl::compute3DCentroid(*octo_tri_no_ground, *iter, temp_centroid);
    		pcl::compute3DCentroid(*pcl_octo_tri, *iter, temp_centroid);
    		float curr2cent = normalize(temp_centroid[0] - curr_pose.pose.position.x, 
							    									temp_centroid[1] - curr_pose.pose.position.y, 
							    									temp_centroid[2] - curr_pose.pose.position.z);
    		if(compare_centroid_dist<0){compare_centroid_dist = (curr2cent*curr2cent/iter->indices.size());}
    		else if(compare_centroid_dist>(curr2cent*curr2cent/iter->indices.size())){compare_centroid_dist = curr2cent*curr2cent/iter->indices.size(); second_idx = closest_idx; closest_idx = id;}
    		// std::cout << "distance: " << curr2cent << std::endl;
    		// std::cout << "num: " << iter->indices.size() << std::endl;
    		// std::cout << "distance/num: " << curr2cent/iter->indices.size() << std::endl;
	    }
	    // std::cout << "closest idx: " << closest_idx << std::endl;
	    // std::cout << "second idx: " << second_idx << std::endl;
	    if(escape){*closest_indices = cluster_indices[second_idx];}
	    else{*closest_indices = cluster_indices[closest_idx];}

	    pcl::ExtractIndices<pcl::PointXYZ> extract;
    	pcl::PointCloud<pcl::PointXYZ>::Ptr close_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    	// extract.setInputCloud (octo_tri_no_ground);
    	extract.setInputCloud (pcl_octo_tri);
    	extract.setIndices (closest_indices);
		extract.filter (*close_cloud);
		std::cout << "closest cloud size : " << close_cloud->size() << std::endl;
		closest_tri_pub.publish(cloud2msg(*close_cloud, "map"));

		// pcl::PointCloud<pcl::PointXYZ>::Ptr close_cloud_down(new pcl::PointCloud<pcl::PointXYZ> ());
		// pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
		// voxel_filter.setInputCloud(close_cloud);
		// voxel_filter.setLeafSize(down_size, down_size, down_size);
		// voxel_filter.filter(*close_cloud_down);
		// std::cout << "down sampled closest cloud size : " << close_cloud_down->size() << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_octo_occu (new pcl::PointCloud<pcl::PointXYZ>);
		*pcl_octo_occu = cloudmsg2cloud(octo_occu);
		std::cout << "occupied cloud size : " << pcl_octo_occu->size() << std::endl;

		kdtree.setInputCloud(pcl_octo_occu);

		// pcl::PointCloud<pcl::PointNormal> reference_points(octo_new_down->size());
		candi.request.offset_distance = offset_distance;
		int idx = -1;

		normals_marker.markers.clear();
		marker_num = 0;

		// for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = octo_new_down->begin(); it!=octo_new_down->end(); ++it, ++idx){
		#pragma omp parallel for
		for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = close_cloud->begin(); it!=close_cloud->end(); ++it, ++idx){
		// for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = close_cloud_down->begin(); it!=close_cloud_down->end(); ++it, ++idx){
			pcl::PointXYZ searchPoint;
		    searchPoint.x = it->x;
		    searchPoint.y = it->y;
		    searchPoint.z = it->z;

		    std::cout << "searchPoint: (" << searchPoint.x << ", " << searchPoint.y << ", " << searchPoint.z << ")" << std::endl;

		    std::vector<int> pointIdxNKNSearch(num_knn);
		    std::vector<float> pointNKNSquaredDistance(num_knn);

		    if (kdtree.nearestKSearch(searchPoint, num_knn, pointIdxNKNSearch, pointNKNSquaredDistance)>0)
		    {
		    	pcl::PointCloud<pcl::PointXYZ> temp_near;
		    	// pcl::PointXYZ temp_centroid = pcl::PointXYZ(pcl_octo_new->points[pointIdxNKNSearch[0]].x,
		    	// 								 			pcl_octo_new->points[pointIdxNKNSearch[0]].y,
		    	// 								 			pcl_octo_new->points[pointIdxNKNSearch[0]].z);
				Eigen::Matrix3f covariance_matrix;
		    	Eigen::Vector4f xyz_centroid;
		    	Eigen::Vector4f plane_parameters;
		    	float curvature;


		    	#pragma omp parallel for
		    	for (size_t i = 0; i < num_knn; ++i){
			    	// std::cout << "pointIdxNKNSearch: (" << pcl_octo_occu->points[pointIdxNKNSearch[i]].x << ", " 
			    	// 									<< pcl_octo_occu->points[pointIdxNKNSearch[i]].y << ", " 
			    	// 									<< pcl_octo_occu->points[pointIdxNKNSearch[i]].z << ")" << std::endl;
		    		temp_near.push_back(pcl::PointXYZ(pcl_octo_occu->points[pointIdxNKNSearch[i]].x,
		    									 	  pcl_octo_occu->points[pointIdxNKNSearch[i]].y,
		    									 	  pcl_octo_occu->points[pointIdxNKNSearch[i]].z));
		    	}

				pcl::computeMeanAndCovarianceMatrix (temp_near, covariance_matrix, xyz_centroid);
				pcl::solvePlaneParameters(covariance_matrix, xyz_centroid, plane_parameters, curvature);


				// std::cout << "num_knn: " << num_knn <<std::endl;
				// std::cout << "temp_near: " << temp_near.size() <<std::endl;
				// std::cout << "num: " << num <<std::endl;

				// pcl::PointNormal temp_ref = pcl::PointNormal(octo_new[pointIdxNKNSearch[0]].x,
				// 											 octo_new[pointIdxNKNSearch[0]].y,
				// 											 octo_new[pointIdxNKNSearch[0]].z,
				// 											 n_x, n_y, n_z, curv);

				// reference_points.points[idx] = temp_ref;
				geometry_msgs::Point ref_point;
				geometry_msgs::Vector3 ref_normal;

				ref_point.x = pcl_octo_occu->points[pointIdxNKNSearch[0]].x;
				ref_point.y = pcl_octo_occu->points[pointIdxNKNSearch[0]].y;
				ref_point.z = pcl_octo_occu->points[pointIdxNKNSearch[0]].z;

				std::cout << "ref_points: " << pcl_octo_occu->points[pointIdxNKNSearch[0]].x << "," 
											<< pcl_octo_occu->points[pointIdxNKNSearch[0]].y << "," 
											<< pcl_octo_occu->points[pointIdxNKNSearch[0]].z << std::endl;
				
				visualization_msgs::Marker normal_marker;
				normal_marker.header = curr_pose.header;
				normal_marker.header.frame_id = "map";
				normal_marker.header.stamp = ros::Time::now();
				normal_marker.ns = "normal_vec";
				normal_marker.id = marker_num;
				normal_marker.type = visualization_msgs::Marker::ARROW;
				normal_marker.action = visualization_msgs::Marker::ADD;
				normal_marker.scale.x =0.15;
				normal_marker.scale.y =0.15;
				normal_marker.scale.z =0.15;
				normal_marker.color.r = 0.7;
				normal_marker.color.g = 0.1;
				normal_marker.color.b = 0.2;
				normal_marker.color.a = 0.5;
				normal_marker.lifetime = ros::Duration();
				// normal_marker.pose.position.x = 0;
			 //    normal_marker.pose.position.y = 0;
			 //    normal_marker.pose.position.z = 0;
			 //    normal_marker.pose.orientation.x = 0.0;
			 //    normal_marker.pose.orientation.y = 0.0;
			 //    normal_marker.pose.orientation.z = 0.0;
			 //    normal_marker.pose.orientation.w = 1.0;
				marker_num++;
				auto normal_mag = normalize(plane_parameters[0],plane_parameters[1],plane_parameters[2]);

				normal_marker.points.push_back(ref_point);
				geometry_msgs::Point added_ref_point;
				added_ref_point.x = ref_point.x + plane_parameters[0]/normal_mag;
				added_ref_point.y = ref_point.y + plane_parameters[1]/normal_mag;
				added_ref_point.z = ref_point.z + plane_parameters[2]/normal_mag;
				normal_marker.points.push_back(added_ref_point);

				normals_marker.markers.push_back(normal_marker);

	      if(isnan(plane_parameters[0]) == 0 &&
			 isnan(plane_parameters[1]) == 0 &&
			 isnan(plane_parameters[2]) == 0 &&
			 isnan(ref_point.x) == 0 &&
			 isnan(ref_point.y) == 0 &&
			 isnan(ref_point.z) == 0)
	      {
					ref_normal.x = plane_parameters[0]/normal_mag;
					ref_normal.y = plane_parameters[1]/normal_mag;
					ref_normal.z = plane_parameters[2]/normal_mag;
					candi.request.ref_points.push_back(ref_point);
					candi.request.ref_normals.push_back(ref_normal);
	      }
	    }
		}
	}
	else
	{
		std::cout << "NO NEW MAP!!!"<< std::endl;
	}
	std::cout << "ref_points size: " << candi.request.ref_points.size() << std::endl;
	client_collision_check.call(candi);
	// if(candi.response.validity){}
	remain_ref_points = candi.response.remain_ref_points;
	candidate = candi.response.candidate;
	candidate_normals = candi.response.candidate_normals;

	int path_size = candidate.poses.size();

	nav_msgs::Path local_path;
	local_path.poses.push_back(curr_pose);

	if(path_size==0){
		std::cout << "no path" << std::endl;
		// std::random_device rd;
		// std::mt19937 gen(rd());
		// std::uniform_int_distribution<int> dis(0, 99);
		// n_cpp::RayCast clear_z;
		// geometry_msgs::PoseStamped rand_end;
		// double x_dir = dis(gen)-99.0/2.0;
		// double y_dir = dis(gen)-99.0/2.0;
		// double z_dir = dis(gen)-99.0/2.0;
		// rand_end.pose.position.x = curr_pose.pose.position.x+x_dir/normalize(x_dir, y_dir, z_dir)*octomap_resolution*3.0;
		// rand_end.pose.position.y = curr_pose.pose.position.y+y_dir/normalize(x_dir, y_dir, z_dir)*octomap_resolution*3.0;
		// rand_end.pose.position.z = curr_pose.pose.position.z+z_dir/normalize(x_dir, y_dir, z_dir)*octomap_resolution*3.0;
		// clear_z.request.start = curr_pose.pose.position;
		// clear_z.request.end = rand_end.pose.position;
		// client_ray_cast.call(clear_z);
		// nav_msgs::Path local_path;
		// if(clear_z.response.visible){local_path.poses.push_back(rand_end);}
		// n_cpp::NextStep::Response refTripleRawPath;
		// refTripleRawPath.ref_triple = remain_ref_points;
		// refTripleRawPath.raw_path = local_path;
		// return refTripleRawPath;
	}
	else{
		int check_idx[path_size];
		std::vector<int> align_idx;

		std::cout << "candi size: " << path_size << std::endl;

		#pragma omp parallel for
		for(int idx = 0; idx<path_size ; ++idx){
			check_idx[idx] = idx;
		}
		int count = 0;
		while(count<path_size){
			if(align_idx.size() == 0){
				int start_idx;
				double min_dist;
				#pragma omp parallel for
				for(int idx = 0; idx<path_size ; ++idx){
					double start_dist = normalize(candidate.poses[idx].pose.position.x - curr_pose.pose.position.x,
																    	  candidate.poses[idx].pose.position.y - curr_pose.pose.position.y,
																    	  candidate.poses[idx].pose.position.z - curr_pose.pose.position.z);
					if(idx == 0) {
						min_dist = start_dist;
						start_idx = 0;
					}
					else{
						if(min_dist > start_dist){
							min_dist = start_dist;
							start_idx = idx;
						}
					}
				}
				align_idx.push_back(start_idx);
				check_idx[start_idx] = -1;
				std::cout << "align step1 end"<< std::endl;
				std::cout << "check_idx: "<<check_idx<< std::endl;
				std::cout << "align_idx back:"<< align_idx.back() <<std::endl;
				count++;
			}
			else{
				int min_idx;
				int init = 0;
				double min_dist;
				#pragma omp parallel for
				for(int idx = 0; idx<path_size ; ++idx){
					if(check_idx[idx] != -1){
						// std::cout << "check idx: " << idx << std::endl;
						double temp_dist = normalize(candidate.poses[idx].pose.position.x - candidate.poses[align_idx.back()].pose.position.x,
																	   	 	 candidate.poses[idx].pose.position.y - candidate.poses[align_idx.back()].pose.position.y,
																	   		 candidate.poses[idx].pose.position.z - candidate.poses[align_idx.back()].pose.position.z);
						if(init == 0) {
							min_dist = temp_dist;
							min_idx = idx;
							++init;
						}
						else{
							if(min_dist > temp_dist){
								min_dist = temp_dist;
								min_idx = idx;
							}
						}
					}
				}
				// align_idx.push_back(min_idx);
				// check_idx[min_idx] = -1;

				check_idx[min_idx] = -1;
				count++;
				std::cout << "count: " << count << std::endl;
				if(path_min<min_dist&&min_dist<path_max){
					std::cout << "path_min<min_dist<path_max: " << path_min<<","<<min_dist<<","<<path_max<< std::endl;
					align_idx.push_back(min_idx);
				}
				else{
					std::cout << "abandoned" <<std::endl;
					// path_size = align_idx.size();
				}
			}
		}


		
		bool clear = true;
		n_cpp::RayCast clear_path;
		clear_path.request.start = curr_pose.pose.position;
		clear_path.request.end = candidate.poses[align_idx[0]].pose.position;
		// ROS_ERROR("DUCKDUCK %d", align_idx.size());
		client_ray_cast.call(clear_path);
		geometry_msgs::PoseStamped pathInit;
		
		if(clear_path.response.visible == false)
		{
			clear = false;
			if(curr_pose.pose.position.z<candidate.poses[align_idx[0]].pose.position.z)
			{
				n_cpp::RayCast clear_z;
				geometry_msgs::PoseStamped curr_z_end;
				curr_z_end = curr_pose;
				curr_z_end.pose.position.z = candidate.poses[align_idx[0]].pose.position.z;
				clear_z.request.start = curr_pose.pose.position;
				clear_z.request.end = curr_z_end.pose.position;
				client_ray_cast.call(clear_z);
				if(clear_z.response.visible == false){
					n_cpp::RayCast clear_plane;
					geometry_msgs::PoseStamped curr_plane_end;
					curr_plane_end = curr_pose;
					curr_plane_end.pose.position.x = curr_pose.pose.position.x + (curr_pose.pose.position.x - candidate.poses[align_idx[0]].pose.position.x)/normalize(curr_pose.pose.position.x - candidate.poses[align_idx[0]].pose.position.x, curr_pose.pose.position.y - candidate.poses[align_idx[0]].pose.position.y)*octomap_resolution*1.4;
					curr_plane_end.pose.position.y = curr_pose.pose.position.y + (curr_pose.pose.position.y - candidate.poses[align_idx[0]].pose.position.y)/normalize(curr_pose.pose.position.x - candidate.poses[align_idx[0]].pose.position.x, curr_pose.pose.position.y - candidate.poses[align_idx[0]].pose.position.y)*octomap_resolution*1.4;
					clear_plane.request.start = curr_pose.pose.position;
					clear_plane.request.end = curr_plane_end.pose.position;
					client_ray_cast.call(clear_plane);
					if(clear_plane.response.visible==false){std::cout<<"Stucked"<<std::endl;}
					else{
						n_cpp::RayCast clear_zz;
						geometry_msgs::PoseStamped curr_zz_end;
						curr_zz_end = curr_plane_end;
						curr_zz_end.pose.position.z = candidate.poses[align_idx[0]].pose.position.z;
						clear_zz.request.start = curr_plane_end.pose.position;
						clear_zz.request.end = curr_zz_end.pose.position;
						client_ray_cast.call(clear_zz);
						if(clear_zz.response.visible){local_path.poses.push_back(curr_plane_end);local_path.poses.push_back(curr_zz_end);}
					}
				}
				else{local_path.poses.push_back(curr_z_end);}
			}
			else{
				n_cpp::RayCast clear_z;
				geometry_msgs::PoseStamped curr_z_end;
				curr_z_end = curr_pose;
				curr_z_end.pose.position.z = candidate.poses[align_idx[0]].pose.position.z;
				clear_z.request.start = curr_pose.pose.position;
				clear_z.request.end = curr_z_end.pose.position;
				client_ray_cast.call(clear_z);
				if(clear_z.response.visible == false){
					n_cpp::RayCast clear_x;
					geometry_msgs::PoseStamped curr_x_end;
					curr_x_end = curr_pose;
					curr_x_end.pose.position.x = candidate.poses[align_idx[0]].pose.position.x;
					clear_x.request.start = curr_pose.pose.position;
					clear_x.request.end = curr_x_end.pose.position;
					client_ray_cast.call(clear_x);
					if(clear_x.response.visible == false){
						n_cpp::RayCast clear_y;
						geometry_msgs::PoseStamped curr_y_end;
						curr_y_end = curr_pose;
						curr_y_end.pose.position.y = candidate.poses[align_idx[0]].pose.position.y;
						clear_y.request.start = curr_pose.pose.position;
						clear_y.request.end = curr_y_end.pose.position;
						client_ray_cast.call(clear_y);
						if(clear_y.response.visible == false)
						{
							n_cpp::RayCast clear_plane;
							geometry_msgs::PoseStamped curr_plane_end;
							curr_plane_end = curr_pose;
							curr_plane_end.pose.position.x = curr_pose.pose.position.x + (curr_pose.pose.position.x - candidate.poses[align_idx[0]].pose.position.x)/normalize(curr_pose.pose.position.x - candidate.poses[align_idx[0]].pose.position.x, curr_pose.pose.position.y - candidate.poses[align_idx[0]].pose.position.y)*octomap_resolution*1.4;
							curr_plane_end.pose.position.y = curr_pose.pose.position.y + (curr_pose.pose.position.y - candidate.poses[align_idx[0]].pose.position.y)/normalize(curr_pose.pose.position.x - candidate.poses[align_idx[0]].pose.position.x, curr_pose.pose.position.y - candidate.poses[align_idx[0]].pose.position.y)*octomap_resolution*1.4;
							clear_plane.request.start = curr_pose.pose.position;
							clear_plane.request.end = curr_plane_end.pose.position;
							client_ray_cast.call(clear_plane);
							if(clear_plane.response.visible==false){std::cout<<"Stucked"<<std::endl;}
							else{local_path.poses.push_back(curr_plane_end);}
						}
						else{local_path.poses.push_back(curr_y_end);}
					}
					else{local_path.poses.push_back(curr_x_end);}
				}
				else{
					local_path.poses.push_back(curr_z_end);
					n_cpp::RayCast clear_x;
					geometry_msgs::PoseStamped curr_x_end;
					curr_x_end = curr_z_end;
					curr_x_end.pose.position.x = candidate.poses[align_idx[0]].pose.position.x;
					clear_x.request.start = curr_z_end.pose.position;
					clear_x.request.end = curr_x_end.pose.position;
					client_ray_cast.call(clear_x);
					if(clear_x.response.visible == false){
						n_cpp::RayCast clear_y;
						geometry_msgs::PoseStamped curr_y_end;
						curr_y_end = curr_z_end;
						curr_y_end.pose.position.y = candidate.poses[align_idx[0]].pose.position.y;
						clear_y.request.start = curr_z_end.pose.position;
						clear_y.request.end = curr_y_end.pose.position;
						client_ray_cast.call(clear_y);
						if(clear_y.response.visible == false)
						{
							n_cpp::RayCast clear_plane;
							geometry_msgs::PoseStamped curr_plane_end;
							curr_plane_end = curr_z_end;
							curr_plane_end.pose.position.x = curr_z_end.pose.position.x + (curr_z_end.pose.position.x - candidate.poses[align_idx[0]].pose.position.x)/normalize(curr_z_end.pose.position.x - candidate.poses[align_idx[0]].pose.position.x, curr_pose.pose.position.y - candidate.poses[align_idx[0]].pose.position.y)*octomap_resolution*1.4;
							curr_plane_end.pose.position.y = curr_z_end.pose.position.y + (curr_z_end.pose.position.y - candidate.poses[align_idx[0]].pose.position.y)/normalize(curr_z_end.pose.position.x - candidate.poses[align_idx[0]].pose.position.x, curr_pose.pose.position.y - candidate.poses[align_idx[0]].pose.position.y)*octomap_resolution*1.4;
							clear_plane.request.start = curr_z_end.pose.position;
							clear_plane.request.end = curr_plane_end.pose.position;
							client_ray_cast.call(clear_plane);
							if(clear_plane.response.visible==false){std::cout<<"Stucked"<<std::endl;}
							else{local_path.poses.push_back(curr_plane_end);}
						}
						else{
							local_path.poses.push_back(curr_y_end);
						}
					}
					else{local_path.poses.push_back(curr_x_end);}
				}
			}
		}

		// local_path = ;
		
		if(clear && align_idx.size()>0)
		{
			std::vector<int>::iterator it;
			#pragma omp parallel for
			for (it = align_idx.begin() ; it != align_idx.end(); it++){
				geometry_msgs::PoseStamped posepose = candidate.poses[*it];
				geometry_msgs::PoseStamped prevpose = local_path.poses.back();
				posepose.header.frame_id = "map";
				n_cpp::RayCast each_path;
				each_path.request.start = prevpose.pose.position;
				each_path.request.end = posepose.pose.position;
				client_ray_cast.call(each_path);
				bool each_result = clear_path.response.visible;
				if(each_result){local_path.poses.push_back(posepose);}
			}
			// if(local_path.poses.size()==1){local_path.poses.erase(local_path.poses.begin());}
			// std::cout << "clear local_path: " << local_path.poses.size() << std::endl;
		}
	}
	if(correction && local_path.poses.size() > 1){
		float distance2start = normalize(curr_pose.pose.position.x-local_path.poses[1].pose.position.x,
										 curr_pose.pose.position.y-local_path.poses[1].pose.position.y,
									 	 curr_pose.pose.position.z-local_path.poses[1].pose.position.z);
		if(distance2start > 1.0){
			geometry_msgs::Point feature;
			feature.x = 0;
			feature.y = 0;
			feature.z = 0;
			geometry_msgs::PoseStamped correction_wp;
			correction_wp.pose.position.x = (curr_pose.pose.position.x+local_path.poses[1].pose.position.x)/2.0;
			correction_wp.pose.position.y = (curr_pose.pose.position.y+local_path.poses[1].pose.position.y)/2.0;
			correction_wp.pose.position.z = (curr_pose.pose.position.z+local_path.poses[1].pose.position.z)/2.0;
			tf::Quaternion quat;
	        quat.setRPY(0, 0, -atan2(feature.y-curr_pose.pose.position.y, feature.x-curr_pose.pose.position.x));
	        quat.normalize();
	        correction_wp.pose.orientation.x = quat.getX();
	        correction_wp.pose.orientation.y = quat.getY();
	        correction_wp.pose.orientation.z = quat.getZ();
	        correction_wp.pose.orientation.w = quat.getW();
	        local_path.poses[0] = correction_wp;
		}
	}
	std::cout << "local_path: " << local_path.poses.size() << std::endl;
	n_cpp::NextStep::Response refTripleRawPath;
	refTripleRawPath.ref_triple = remain_ref_points;
	refTripleRawPath.raw_path = local_path;
	return refTripleRawPath;
}

