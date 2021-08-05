#include "path_gen.hpp"

nav_msgs::Path N_path::path_gen()
{
	////Generate kd-tree
	// pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	nanoflann::KdTreeFLANN<pcl::PointXYZ> kdtree;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_octo_new(new pcl::PointCloud<pcl::PointXYZ> ());
	*pcl_octo_new = cloudmsg2cloud(octo_new);
	n_cpp::collision_check candi;

	if(pcl_octo_new->points.size()>0){
		kdtree.setInputCloud(pcl_octo_new);

		pcl::PointCloud<pcl::PointXYZ>::Ptr octo_new_no_ground(new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PassThrough<pcl::PointXYZ> full_filter;
		full_filter.setInputCloud(pcl_octo_new);
		full_filter.setFilterFieldName("z");
		full_filter.setFilterLimits(ground_thresh, 25.0);
		full_filter.filter(*octo_new_no_ground);

		pcl::PointCloud<pcl::PointXYZ>::Ptr octo_new_down(new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
		voxel_filter.setInputCloud(octo_new_no_ground);
		voxel_filter.setLeafSize(down_size, down_size, down_size);
		voxel_filter.filter(*octo_new_down);

		// pcl::PointCloud<pcl::PointNormal> reference_points(octo_new_down->size());
		candi.request.offset_distance = offset_distance;
		int idx = -1;

		normals_marker.markers.clear();
		marker_num = 0;

		for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = octo_new_down->begin(); it!=octo_new_down->end(); ++it){
			++idx;
			pcl::PointXYZ searchPoint;
		    searchPoint.x = it->x;
		    searchPoint.y = it->y;
		    searchPoint.z = it->z;

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

		    	for (size_t i = 0; i < num_knn; ++i){
		    		temp_near.push_back(pcl::PointXYZ(pcl_octo_new->points[pointIdxNKNSearch[i]].x,
		    									 	  pcl_octo_new->points[pointIdxNKNSearch[i]].y,
		    									 	  pcl_octo_new->points[pointIdxNKNSearch[i]].z));
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

				ref_point.x = pcl_octo_new->points[pointIdxNKNSearch[0]].x;
				ref_point.y = pcl_octo_new->points[pointIdxNKNSearch[0]].y;
				ref_point.z = pcl_octo_new->points[pointIdxNKNSearch[0]].z;

				std::cout << "ref_points: " << pcl_octo_new->points[pointIdxNKNSearch[0]].x << "," 
											<< pcl_octo_new->points[pointIdxNKNSearch[0]].y << "," 
											<< pcl_octo_new->points[pointIdxNKNSearch[0]].z << std::endl;
				
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
	candidate = candi.response.candidate;
	candidate_normals = candi.response.candidate_normals;

	int path_size = candidate.poses.size();
	int check_idx[path_size];
	std::vector<int> align_idx;
	
	std::cout << "candi size: " << path_size << std::endl;

	for(int idx = 0; idx==path_size ; ++idx){
		check_idx[idx] = idx;
	}
	while(align_idx.size()<path_size){

		if(align_idx.size() == 0){
			int start_idx;
			double min_dist;
			for(int idx = 0; idx<path_size ; ++idx){
				double start_dist = pow(pow(candidate.poses[idx].pose.position.x - octo_pose.pose.position.x,2) +
									    pow(candidate.poses[idx].pose.position.y - octo_pose.pose.position.y,2) +
									    pow(candidate.poses[idx].pose.position.z - octo_pose.pose.position.z,2), 0.5);
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
		}
		else{
			int min_idx;
			int count = 0;
			double min_dist;
			for(int idx = 0; idx<path_size ; ++idx){
				if(check_idx[idx] != -1){
					double temp_dist = pow(pow(candidate.poses[idx].pose.position.x - candidate.poses[align_idx.back()].pose.position.x,2) +
										   pow(candidate.poses[idx].pose.position.y - candidate.poses[align_idx.back()].pose.position.x,2) +
										   pow(candidate.poses[idx].pose.position.z - candidate.poses[align_idx.back()].pose.position.x,2), 0.5);
					if(count == 0) {
						min_dist = temp_dist;
						min_idx = 0;
					}
					else{
						if(min_dist > temp_dist){
							min_dist = temp_dist;
							min_idx = idx;
						}
					}
					++count;
				}
			}
			align_idx.push_back(min_idx);
			check_idx[min_idx] = -1;
			// if(min_dist<path_max){
			// 	std::cout << "abandoned" <<std::endl;
			// 	align_idx.push_back(min_idx);
			// 	check_idx[min_idx] = -1;
			// }
		}
	}
	nav_msgs::Path local_path;
	for (std::vector<int>::iterator it = align_idx.begin() ; it != align_idx.end(); ++it){
		geometry_msgs::PoseStamped posepose = candidate.poses[*it];
		posepose.header.frame_id = "map";
		local_path.poses.push_back(posepose);
	}

	// local_path = candidate; //DUCK if no align 

	std::cout << "local_path: " << local_path.poses.size() << std::endl;
	return local_path;
}