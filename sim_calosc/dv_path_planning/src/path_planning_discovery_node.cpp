#include <ros/ros.h>
#include "path_planner/DiscoveryPathPlanner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planning");
    ROS_INFO("Starting path planning DISCOVERY node");

    DiscoveryPathPlanner path_planner;

    ros::spin();
    return 0;
}