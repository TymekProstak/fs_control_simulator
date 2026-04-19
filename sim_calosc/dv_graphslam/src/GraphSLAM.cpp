#include "GraphSLAM.h"
#include "Utils.h"
#include "nav_msgs/Odometry.h"

GraphSLAM::GraphSLAM(ros::NodeHandle& nh) : nh_(nh), graphManager_(nh), tf_listener_(tf_buffer_)
{
    ROS_INFO("Initializing GraphSLAM...");

    // Load topic and frame configuration from parameter server
    nh_.param<std::string>("topics/odom_input", odom_input_topic_, "/ins/pose");
    nh_.param<std::string>("topics/cones_input", cones_input_topic_, "/dv_cone_detector/camera/cones");
    nh_.param<std::string>("topics/pose_output", pose_output_topic_, "/slam/pose");
    nh_.param<std::string>("topics/cones_output", cones_output_topic_, "/graphslam/cones");
    nh_.param<std::string>("topics/markers_output", markers_output_topic_, "/graphslam/markers");
    nh_.param<std::string>("tf/map_frame", map_frame_, "map");
    nh_.param<std::string>("tf/robot_frame", robot_frame_, "bolide_CoG");

    ROS_INFO("[GraphSLAM] Subscribing to odom: %s", odom_input_topic_.c_str());
    ROS_INFO("[GraphSLAM] Subscribing to cones: %s", cones_input_topic_.c_str());
    ROS_INFO("[GraphSLAM] Publishing pose to: %s", pose_output_topic_.c_str());
    ROS_INFO("[GraphSLAM] Publishing cones to: %s", cones_output_topic_.c_str());
    ROS_INFO("[GraphSLAM] Publishing markers to: %s", markers_output_topic_.c_str());
    ROS_INFO("[GraphSLAM] TF: %s -> %s", map_frame_.c_str(), robot_frame_.c_str());

    setupSubscribers();
    setupPublishers();
}

void GraphSLAM::setupSubscribers()
{
    odom_sub_ = nh_.subscribe(odom_input_topic_, 50, &GraphSLAM::odomCallback, this);
    cones_sub_ = nh_.subscribe(cones_input_topic_, 10, &GraphSLAM::conesCallback, this);
}

void GraphSLAM::setupPublishers()
{
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(markers_output_topic_, 10);
    pose_pub_ = nh_.advertise<nav_msgs::Odometry>(pose_output_topic_, 10);
    cones_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cones_output_topic_, 10);
}

void GraphSLAM::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double yaw = quaternionToYaw(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    // Store latest raw odom — pose is only added to graph when cones arrive
    Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = yaw;
    graphManager_.updateOdom(pose, msg->header.stamp);

    // Publish corrected pose from the same (possibly smoothed) odom stream
    // that backend SLAM uses for lookup / edge construction.
    Pose2D odom_for_publish = pose;
    Pose2D looked_up;
    if (graphManager_.lookupPose(msg->header.stamp, looked_up)) {
        odom_for_publish = looked_up;
    }

    const Pose2D corrected_pose = applyOffset(odom_for_publish, graphManager_.getMapToOdomOffset());
    publishPose(corrected_pose, msg->header.stamp);
    publishTF(corrected_pose.x, corrected_pose.y, corrected_pose.theta, msg->header.stamp);
    ROS_INFO_THROTTLE(1, "[SLAM]: Odometry");
}

void GraphSLAM::conesCallback(const dv_interfaces::Cones::ConstPtr& msg)
{
    // Look up sensor -> robot transform via TF
    const std::string& sensor_frame = msg->header.frame_id;
    double offset_x = 0.0, offset_y = 0.0, offset_yaw = 0.0;

    if (!sensor_frame.empty() && sensor_frame != robot_frame_) {
        try {
            auto tf = tf_buffer_.lookupTransform(
                robot_frame_, sensor_frame, msg->header.stamp, ros::Duration(0.1));
            offset_x = tf.transform.translation.x;
            offset_y = tf.transform.translation.y;
            offset_yaw = quaternionToYaw(
                tf.transform.rotation.x, tf.transform.rotation.y,
                tf.transform.rotation.z, tf.transform.rotation.w);
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5, "[GraphSLAM] TF lookup %s->%s failed: %s",
                              robot_frame_.c_str(), sensor_frame.c_str(), ex.what());
            return; // Cannot transform, skip this message
        }
    }

    const double cos_off = std::cos(offset_yaw);
    const double sin_off = std::sin(offset_yaw);

    // Transform observations from sensor frame to robot frame
    std::vector<Landmark> observations;
    observations.reserve(msg->cones.size());

    for (const auto& c : msg->cones) {
        Landmark lm;
        lm.mean.x() = cos_off * c.x - sin_off * c.y + offset_x;
        lm.mean.y() = sin_off * c.x + cos_off * c.y + offset_y;
        lm.type = c.class_name;
        observations.push_back(lm);
    }

    Pose2D pose_at_capture;
    if (!graphManager_.lookupPose(msg->header.stamp, pose_at_capture)) {
        ROS_WARN_THROTTLE(5, "[GraphSLAM] No odom in buffer for cone timestamp, skipping");
        return;
    }

    // Add a new pose and process observations atomically
    graphManager_.addPoseAndObservations(observations, pose_at_capture);
    
    // Cones will be published after each iteration of the optimizer loop
    // Znowu na pałę do sprawdzenia
    if (graphManager_.optimized_) {
        publishLandmarkMarkers();
        publishCones();
        graphManager_.optimized_ = false;
    }
    ROS_INFO_THROTTLE(1, "[GraphSLAM] Processed %zu cone observations", msg->cones.size());
}

void GraphSLAM::publishPose(const Pose2D& pose, const ros::Time& stamp)
{
    nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.child_frame_id = robot_frame_;
    pose_msg.pose.pose.position.x = pose.x;
    pose_msg.pose.pose.position.y = pose.y;
    pose_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    pose_pub_.publish(pose_msg);
}

void GraphSLAM::publishTF(double x, double y, double yaw, const ros::Time& stamp)
{
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = robot_frame_;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(tf_msg);
}

void GraphSLAM::publishLandmarkMarkers()
{
    visualization_msgs::MarkerArray marker_array;
    const auto landmarks = graphManager_.getConfirmedLandmarks();

    // Clear all previous markers to remove stale ones (e.g. after landmark merging)
    visualization_msgs::Marker delete_all;
    delete_all.header.frame_id = map_frame_;
    delete_all.header.stamp = ros::Time::now();
    delete_all.action = visualization_msgs::Marker::DELETEALL;

    delete_all.ns = "cones";
    marker_array.markers.push_back(delete_all);
    delete_all.ns = "cone_labels";
    marker_array.markers.push_back(delete_all);

    // Publish cone markers
    for (const auto& lm : landmarks) {
        visualization_msgs::Marker cone_marker;
        cone_marker.header.frame_id = map_frame_;
        cone_marker.header.stamp = ros::Time::now();
        cone_marker.ns = "cones";
        cone_marker.id = lm.id;
        cone_marker.type = visualization_msgs::Marker::CYLINDER;
        cone_marker.action = visualization_msgs::Marker::ADD;
        
        cone_marker.pose.position.x = lm.mean.x();
        cone_marker.pose.position.y = lm.mean.y();
        cone_marker.pose.position.z = 0.15; // Half of cone height
        cone_marker.pose.orientation.w = 1.0;
        
        cone_marker.scale.x = 0.228; // Cone diameter
        cone_marker.scale.y = 0.228;
        cone_marker.scale.z = 0.3;   // Cone height
        
        // Color based on cone type (matching dv_slam colors)
        if (lm.type == "blue") {
            cone_marker.color.r = 0.0;
            cone_marker.color.g = 0.5;
            cone_marker.color.b = 1.0;
        } else if (lm.type == "yellow") {
            cone_marker.color.r = 1.0;
            cone_marker.color.g = 1.0;
            cone_marker.color.b = 0.2;
        } else if (lm.type == "orange") {
            cone_marker.color.r = 1.0;
            cone_marker.color.g = 0.5;
            cone_marker.color.b = 0.0;
        } else if (lm.type == "large_orange") {
            cone_marker.color.r = 1.0;
            cone_marker.color.g = 0.5;
            cone_marker.color.b = 0.0;
            cone_marker.scale.z = 0.5; // Larger cone
        } else {
            cone_marker.color.r = 0.8;
            cone_marker.color.g = 0.8;
            cone_marker.color.b = 0.8;
        }
        cone_marker.color.a = 1.0;
        
        marker_array.markers.push_back(cone_marker);
        
        // Add text marker for observation count
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = map_frame_;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "cone_labels";
        text_marker.id = lm.id + 100000; // Offset to avoid ID collision
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        text_marker.pose.position.x = lm.mean.x();
        text_marker.pose.position.y = lm.mean.y();
        text_marker.pose.position.z = 0.5; // Above the cone
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.15; // Text size
        
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        text_marker.text = std::to_string(lm.observation_count);
        
        marker_array.markers.push_back(text_marker);
    }
    
    markers_pub_.publish(marker_array);
}

void GraphSLAM::publishCones()
{
    const auto landmarks = graphManager_.getConfirmedLandmarks();

    pcl::PointCloud<pcl::PointXYZL> cones_pcl;
    for (const auto& lm : landmarks) {
        pcl::PointXYZL p;
        p.x = static_cast<float>(lm.mean.x());
        p.y = static_cast<float>(lm.mean.y());
        p.z = 0.0f;

        // Map cone type string to integer label (matching dv_slam ConeType enum)
        if (lm.type == "orange")             p.label = 0;  // CONE_ORANGE_SMALL
        else if (lm.type == "blue")          p.label = 1;  // CONE_BLUE_LEFT
        else if (lm.type == "yellow")        p.label = 2;  // CONE_YELLOW_RIGHT
        else if (lm.type == "large_orange")  p.label = 3;  // CONE_ORANGE_BIG
        else                                  p.label = 4;  // CONE_UNKNOWN

        cones_pcl.push_back(p);
    }

    sensor_msgs::PointCloud2 cones_msg;
    pcl::toROSMsg(cones_pcl, cones_msg);
    cones_msg.header.frame_id = map_frame_;
    cones_msg.header.stamp = ros::Time::now();
    cones_pub_.publish(cones_msg);
}
