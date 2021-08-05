#include "path_gen.hpp"
void N_path::subs_pix_local_geo(const geometry_msgs::PoseStamped::ConstPtr &msg){curr_pose = *msg;}
void N_path::subs_octo_occu(const sensor_msgs::PointCloud2::ConstPtr &msg){octo_occu = *msg;}