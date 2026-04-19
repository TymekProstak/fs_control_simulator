#include <ros/ros.h>
#include "path_planner/TrackdrivePathPlanner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planning");
    ROS_INFO("Starting path planning TRACKDRIVE node");

    TrackdrivePathPlanner path_planner;

    ros::spin();
    return 0;
}