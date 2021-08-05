#include "octomap.hpp"


sensor_msgs::PointCloud2 N_octomap::cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id) // = "camera_link"
{
	sensor_msgs::PointCloud2 cloud_ROS;
	pcl::toROSMsg(cloud, cloud_ROS);
	cloud_ROS.header.frame_id = frame_id;
	return cloud_ROS;
}

pcl::PointCloud<pcl::PointXYZ> N_octomap::cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudresult(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(cloudmsg, *cloudresult);
	return *cloudresult;
}

octomap::Pointcloud N_octomap::cloudmsg2octocloud(sensor_msgs::PointCloud2 cloudmsg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>());
	*pcl_pc = cloudmsg2cloud(cloudmsg);
	
	octomap::Pointcloud octopc_result;
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pcl_pc->begin(); it!=pcl_pc->end(); ++it){
	  octopc_result.push_back(it->x, it->y, it->z);
	}	
	// pcl_pc->clear();
	pcl_pc.reset(new pcl::PointCloud<pcl::PointXYZ>());
	return octopc_result;
}

octomap::Pointcloud N_octomap::cloud2octocloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn)
{
	int cloudSize = cloudIn->size();
	octomap::Pointcloud octopc_result;

	#pragma omp parallel for num_threads(numberOfCores)
	for (int i = 0; i < cloudSize; ++i){
	  	octopc_result.push_back(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z);
	}
	return octopc_result;
}

octomap::Pointcloud N_octomap::cloud2transOctoCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Matrix4f map_t_sensor)
{
	octomap::Pointcloud octopc_result;
	int cloudSize = cloudIn->size();

	#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
    	const auto &pointFrom = cloudIn->points[i];
    	#pragma omp critical
    	octopc_result.push_back(map_t_sensor(0,0) * pointFrom.x + map_t_sensor(0,1) * pointFrom.y + map_t_sensor(0,2) * pointFrom.z + map_t_sensor(0,3),
    							map_t_sensor(1,0) * pointFrom.x + map_t_sensor(1,1) * pointFrom.y + map_t_sensor(1,2) * pointFrom.z + map_t_sensor(1,3),
    							map_t_sensor(2,0) * pointFrom.x + map_t_sensor(2,1) * pointFrom.y + map_t_sensor(2,2) * pointFrom.z + map_t_sensor(2,3));
    }

	return octopc_result;
}


octomath::Pose6D N_octomap::cvt2octomath(nav_msgs::Odometry curr_odom){
	octomath::Vector3 octo_trans(curr_odom.pose.pose.position.x, curr_odom.pose.pose.position.y, curr_odom.pose.pose.position.z);
	octomath::Quaternion octo_quat(curr_odom.pose.pose.orientation.w, curr_odom.pose.pose.orientation.x, curr_odom.pose.pose.orientation.y, curr_odom.pose.pose.orientation.z);
	octomath::Pose6D cvt_result(octo_trans, octo_quat);
	return cvt_result;
}
octomath::Pose6D N_octomap::cvt2octomath(geometry_msgs::PoseStamped curr_pose){
	octomath::Quaternion octo_quat(curr_pose.pose.orientation.w, curr_pose.pose.orientation.x, curr_pose.pose.orientation.y, curr_pose.pose.orientation.z);
	octomath::Vector3 octo_euler = octo_quat.toEuler();
	float x = curr_pose.pose.position.x;
	float y = curr_pose.pose.position.y;
	float z = curr_pose.pose.position.z;
	double roll = octo_euler.x();
	double pitch = octo_euler.y();
	double yaw = octo_euler.z();
	octomath::Pose6D cvt_result(x, y, z, roll, pitch, yaw);
	// octomath::Pose6D cvt_result;
	return cvt_result;
}

void N_octomap::cvt_octo2pubpc(octomap::OcTree *octree)
{
	sensor_msgs::PointCloud2 pub_octo;
	sensor_msgs::PointCloud2 pub_octo_free;
	pcl::PointCloud<pcl::PointXYZ>::Ptr octo_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr free_octo_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
	for (octomap::OcTree::iterator it=octree->begin(); it!=octree->end(); ++it){
		if(octree->isNodeOccupied(*it))
		{
			octo_pcl_pub->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
		}
		else
		{
			free_octo_pcl_pub->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
		}
	}
	pub_octo = cloud2msg(*octo_pcl_pub, "map");
	pub_octo_free = cloud2msg(*free_octo_pcl_pub, "map");
	octo_occu_pub.publish(pub_octo);
	octo_free_pub.publish(pub_octo_free);
}

bool N_octomap::is_this_triplepoint(octomap::OcTree *octree, octomap::OcTreeKey tripoint_candidate)
{
	double x_cand=octree->keyToCoord(tripoint_candidate).x(), y_cand=octree->keyToCoord(tripoint_candidate).y(), z_cand=octree->keyToCoord(tripoint_candidate).z();
	if(z_cand<ground_thresh){return false;}
	// else if(z_cand<ground_thresh){return false;}
	else
	{
		std::vector<std::vector<std::vector<int>>> cand_sliding_window(3);//unknown == -1, occ == 1, free = 0

		#pragma omp parallel for
		for(int i=0; i<3; ++i)
		{
			std::vector<std::vector<int>> temp_33_mat(3);
			#pragma omp parallel for
			for(int j=0; j<3; ++j)
			{
				std::vector<int> temp_row(3);
				#pragma omp parallel for
				for(int k=0; k<3; ++k)
				{
					double x = x_cand + octomap_resolution * (i-1);
					double y = y_cand + octomap_resolution * (j-1);
					double z = z_cand + octomap_resolution * (k-1);

					octomap::OcTreeNode*  node = octree -> search(x, y, z, 0);

					if(node){
						if (m_octree->isNodeOccupied(node)){temp_row[k] = 1;}//occupied
						else                               {temp_row[k] = 0;}//free
					}
					else{temp_row[k] = -1;}//unknown
				}
				temp_33_mat[j] = temp_row;
			}
			cand_sliding_window[i] = temp_33_mat;
		}
		bool un2occ=false, occ2free=false, free2un=false, result;
		#pragma omp parallel for
		for(int i=0; i<2; ++i)
		{
			#pragma omp parallel for
			for(int j=0; j<3; ++j)
			{
				#pragma omp parallel for
				for(int k=0; k<3; ++k)
				{
					if(cand_sliding_window[i][j][k] == -1)
					{
						if     (cand_sliding_window[i+1][j][k] == 0){free2un = true;}
						else if(cand_sliding_window[i+1][j][k] == 1){un2occ = true;}
					}
					else if(cand_sliding_window[i][j][k] == 0)
					{
						if     (cand_sliding_window[i+1][j][k] == -1){free2un = true;}
						else if(cand_sliding_window[i+1][j][k] == 1){occ2free = true;}
					}
					else
					{
						if     (cand_sliding_window[i+1][j][k] == 0){occ2free = true;}
						else if(cand_sliding_window[i+1][j][k] == -1){un2occ = true;}
					}
				}
			}
		}

		#pragma omp parallel for
		for(int i=0; i<3; ++i)
		{
			#pragma omp parallel for
			for(int j=0; j<2; ++j)
			{
				#pragma omp parallel for
				for(int k=0; k<3; ++k)
				{
					if(cand_sliding_window[i][j][k] == -1)
					{
						if     (cand_sliding_window[i][j+1][k] == 0){free2un = true;} //  ROS_ERROR("free2un");
						else if(cand_sliding_window[i][j+1][k] == 1){un2occ = true;} // ROS_ERROR("free2un");
					}
					else if(cand_sliding_window[i][j][k] == 0)
					{
						if     (cand_sliding_window[i][j+1][k] == -1){free2un = true;} // ROS_ERROR("free2un");
						else if(cand_sliding_window[i][j+1][k] == 1){occ2free = true;} // ROS_ERROR("occ2free");
					}
					else
					{
						if     (cand_sliding_window[i][j+1][k] == 0){occ2free = true;} // ROS_ERROR("occ2free");
						else if(cand_sliding_window[i][j+1][k] == -1){un2occ = true;} // ROS_ERROR("un2occ");
					}
				}
			}
		}
		#pragma omp parallel for
		for(int i=0; i<3; ++i)
		{
			#pragma omp parallel for
			for(int j=0; j<3; ++j)
			{
				#pragma omp parallel for
				for(int k=0; k<2; ++k)
				{
					if(cand_sliding_window[i][j][k] == -1)
					{
						if     (cand_sliding_window[i][j][k+1] == 0){free2un = true;}
						else if(cand_sliding_window[i][j][k+1] == 1){un2occ = true;}
					}
					else if(cand_sliding_window[i][j][k] == 0)
					{
						if     (cand_sliding_window[i][j][k+1] == -1){free2un = true;}
						else if(cand_sliding_window[i][j][k+1] == 1){occ2free = true;}
					}
					else
					{
						if     (cand_sliding_window[i][j][k+1] == 0){occ2free = true;}
						else if(cand_sliding_window[i][j][k+1] == -1){un2occ = true;}
					}
				}
			}
		}

		if(cand_sliding_window[1][1][1]!=1){result = false;}
		// else if (cand_sliding_window[0][1][1]!=0 && 
		// 		 cand_sliding_window[2][1][1]!=0 && 
		// 		 cand_sliding_window[1][1][0]!=0 && 
		// 		 cand_sliding_window[1][1][2]!=0) {result = false;}
		// else if (cand_sliding_window[1][0][1]!=0 && 
		// 		 cand_sliding_window[1][2][1]!=0 && 
		// 		 cand_sliding_window[1][1][0]!=0 && 
		// 		 cand_sliding_window[1][1][2]!=0) {result = false;}
		// else if (cand_sliding_window[0][1][1]!=0 && 
		// 		 cand_sliding_window[2][1][1]!=0 && 
		// 		 cand_sliding_window[1][0][1]!=0 && 
		// 		 cand_sliding_window[1][2][1]!=0) {result = false;}
		else{result = un2occ&&occ2free&&free2un;}
		// std::cout << "result :" << result << std::endl;
		return result;
	}
}

float N_octomap::normalize(float a, float b)						{float sum;  float result;  sum = pow(a,2) + pow(b,2); result = pow(sum,0.5); return result;}
float N_octomap::normalize(float a, float b, float c)				{float sum;  float result;  sum = pow(a,2) + pow(b,2) + pow(c,2); result = pow(sum,0.5); return result;}
float N_octomap::normalize(float a, float b, float c, float d)		{float sum;  float result;  sum = pow(a,2) + pow(b,2) + pow(c,2) + pow(d,2); result = pow(sum,0.5); return result;}
double N_octomap::normalize(double a, double b)					{double sum; double result; sum = pow(a,2) + pow(b,2); result = pow(sum,0.5); return result;}
double N_octomap::normalize(double a, double b, double c)			{double sum; double result; sum = pow(a,2) + pow(b,2) + pow(c,2); result = pow(sum,0.5); return result;}
double N_octomap::normalize(double a, double b, double c, double d){double sum; double result; sum = pow(a,2) + pow(b,2) + pow(c,2) + pow(d,2); result = pow(sum,0.5); return result;}