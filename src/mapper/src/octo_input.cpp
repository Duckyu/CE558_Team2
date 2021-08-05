#include "octomap.hpp"

bool N_octomap::data_input(octomap::OcTree *octree, sensor_msgs::PointCloud2 input, int sensor_direction)
{
	bool result = true;
	Eigen::Matrix4f map_t_sensor;
	switch(sensor_direction){
	case FRONT:{map_t_sensor = map_t_body * body_t_front;}
		break;
	case UP:{map_t_sensor = map_t_body * body_t_up;}
		break;
	// case R:
	// 	map_t_sensor = map_t_body * body_t_right;
	// 	break;
	default:{std::cout << "re-check direction!" << std::endl;}
		break;
	}
	if (input.point_step>0){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>());
		*cloudIn = cloudmsg2cloud(input);
		octomap::Pointcloud transOcto = cloud2transOctoCloud(cloudIn, map_t_sensor);
		// std::cout << "data_input size: " << cloudIn->size()<< std::endl;
		octree->insertPointCloud(transOcto, octomap::point3d(map_t_sensor(0,3),map_t_sensor(1,3),map_t_sensor(2,3)), sensor_range);//
	}
	else{
		std::cout << "no input!!!" << std::endl;
		result = false;
	}
	return result;
}

void N_octomap::input_timer(const ros::TimerEvent& event)
{
	// Eigen::Matrix4f map_t_front = map_t_body * body_t_front ;
	if (data_input(m_octree, _front, FRONT)) {}//std::cout << "front data input" << std::endl;
	// Eigen::Matrix4f map_t_up = map_t_body * body_t_up ;
	if (data_input(m_octree, _up, UP)) {}//std::cout << "up data input" << std::endl;
	cvt_octo2pubpc(m_octree);

	// std::cout << "m_octree: " << m_octree->size() << std::endl;
}