#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <dv_interfaces/Cones.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "Utils.h"
#include "GraphManager.h"
#include "ros/publisher.h"

class GraphSLAM
{
public:
    explicit GraphSLAM(ros::NodeHandle& nh);

private:
    // --- ROS Callbacks ---
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void conesCallback(const dv_interfaces::Cones::ConstPtr& msg);

    // --- Helper Functions ---
    void publishTF(double x, double y, double yaw, const ros::Time& stamp);
    void setupSubscribers();
    void setupPublishers();
    void publishPose(const Pose2D&, const ros::Time& stamp);
    void publishLandmarkMarkers();
    void publishCones();

    // --- ROS Handles ---
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cones_sub_;
    ros::Publisher markers_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher cones_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // --- Configuration (loaded from params) ---
    std::string odom_input_topic_;
    std::string cones_input_topic_;
    std::string pose_output_topic_;
    std::string cones_output_topic_;
    std::string markers_output_topic_;
    std::string map_frame_;
    std::string robot_frame_;

    GraphManager graphManager_;
};
