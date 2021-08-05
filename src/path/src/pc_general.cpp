#include "path_gen.hpp"

sensor_msgs::PointCloud2 N_path::cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id) // = "camera_link"
{
	sensor_msgs::PointCloud2 cloud_ROS;
	pcl::toROSMsg(cloud, cloud_ROS);
	cloud_ROS.header.frame_id = frame_id;
	return cloud_ROS;
}

pcl::PointCloud<pcl::PointXYZ> N_path::cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudresult(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(cloudmsg, *cloudresult);
	return *cloudresult;
	// pcl::PointCloud<pcl::PointXYZ> cloudresult;
	// pcl::fromROSMsg(cloudmsg, cloudresult);
	// ROS_ERROR("DUCK trans");
	// return cloudresult;
}

octomap::Pointcloud N_path::cloudmsg2octocloud(sensor_msgs::PointCloud2 cloudmsg)
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

octomap::Pointcloud N_path::cloud2octocloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn)
{
	int cloudSize = cloudIn->size();
	octomap::Pointcloud octopc_result;

	#pragma omp parallel for num_threads(numberOfCores)
	for (int i = 0; i < cloudSize; ++i){
	  	octopc_result.push_back(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z);
	}
	return octopc_result;
}

octomap::Pointcloud N_path::cloud2transOctoCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Matrix4f map_t_sensor)
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


octomath::Pose6D N_path::cvt2octomath(nav_msgs::Odometry curr_odom){
	octomath::Vector3 octo_trans(curr_odom.pose.pose.position.x, curr_odom.pose.pose.position.y, curr_odom.pose.pose.position.z);
	octomath::Quaternion octo_quat(curr_odom.pose.pose.orientation.w, curr_odom.pose.pose.orientation.x, curr_odom.pose.pose.orientation.y, curr_odom.pose.pose.orientation.z);
	octomath::Pose6D cvt_result(octo_trans, octo_quat);
	return cvt_result;
}
octomath::Pose6D N_path::cvt2octomath(geometry_msgs::PoseStamped curr_pose){
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

// Eigen::Vec3f N_path::pose2eigen(geometry_msgs::PoseStamped pose){Eigen::Vec3f result(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z); return result;}
// Eigen::Vec3f N_path::pose2eigen(geometry_msgs::Pose pose){Eigen::Vec3f result(pose.position.x, pose.position.y, pose.position.z); return result;}


float N_path::normalize(float a, float b)						{float sum;  float result;  sum = pow(a,2) + pow(b,2); result = pow(sum,0.5); return result;}
float N_path::normalize(float a, float b, float c)				{float sum;  float result;  sum = pow(a,2) + pow(b,2) + pow(c,2); result = pow(sum,0.5); return result;}
float N_path::normalize(float a, float b, float c, float d)		{float sum;  float result;  sum = pow(a,2) + pow(b,2) + pow(c,2) + pow(d,2); result = pow(sum,0.5); return result;}
double N_path::normalize(double a, double b)					{double sum; double result; sum = pow(a,2) + pow(b,2); result = pow(sum,0.5); return result;}
double N_path::normalize(double a, double b, double c)			{double sum; double result; sum = pow(a,2) + pow(b,2) + pow(c,2); result = pow(sum,0.5); return result;}
double N_path::normalize(double a, double b, double c, double d){double sum; double result; sum = pow(a,2) + pow(b,2) + pow(c,2) + pow(d,2); result = pow(sum,0.5); return result;}