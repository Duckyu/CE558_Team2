#include "src/path.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "n_cpp_path");
    n_path::waypoint_gen waypoint_gen = new n_path::waypoint_gen();

    waypoint_gen->nh.getParam("arrive_distance",waypoint_gen->arrive_distance);
    waypoint_gen->nh.getParam("threshold_yaw",waypoint_gen->threshold_yaw);
    waypoint_gen->nh.getParam("LOS_radius",waypoint_gen->LOS_radius);
    waypoint_gen->nh.getParam("auto_arrive_delay",waypoint_gen->auto_arrive_delay);
    waypoint_gen->nh.getParam("test_mode",waypoint_gen->test_mode);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        waypoint_gen.spinOnce();
    }

    ros::spin();

    return 0;
}