#include <ros/ros.h>
#include "GraphSLAM.h"



int main(int argc, char** argv) 
{
    // Node initialization
    ros::init(argc, argv, "graphslam_node");
    ros::NodeHandle nh("~");
    ROS_INFO("Starting dv_graphslam node");

    GraphSLAM graphslam(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
