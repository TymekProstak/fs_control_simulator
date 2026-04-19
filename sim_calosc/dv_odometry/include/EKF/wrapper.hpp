#pragma once

#include <string>
#include <vector>
#include <cstddef>

#include <ros/ros.h>

#include "dv_interfaces/DV_board.h"
#include "dv_interfaces/Imu.h"
#include "dv_interfaces/Odom_state.h"
#include "dv_interfaces/OdomDebug.h"
#include "dv_interfaces/Steer.h"
#include "dv_interfaces/Control.h"

#include "EKF/EKF.hpp"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

namespace kalman_filter_kinematic
{

class EKFWrapper
{
public:
    EKFWrapper(ros::NodeHandle& nh,
               ros::NodeHandle& pnh,
               const ParamBank& param_bank);

    void spin();

private:
    std::string getRequiredStringParam(const std::string& name) const;
    double getRequiredDoubleParam(const std::string& name) const;
    int getRequiredIntParam(const std::string& name) const;

    void imuCallback(const dv_interfaces::Imu::ConstPtr& msg_ptr);
    void wheelSpeedCallback(const dv_interfaces::DV_board::ConstPtr& msg_ptr);
    void steerCallback(const dv_interfaces::Steer::ConstPtr& msg_ptr);
    void torqueCallback(const dv_interfaces::Control::ConstPtr& msg_ptr);
    void loopTimerCallback(const ros::TimerEvent& event);
    void publishTimerCallback(const ros::TimerEvent& event);
    void InsCallback(const nav_msgs::Odometry::ConstPtr& msg_ptr);

    void finalizeGravityCalibration();
    void publishState();
    void publishOdometry();
    void publish_odom_tf();

    double updateAndFilterSteerDot(double steer_now, const ros::Time& stamp);
    double applySteerDotFilterCascade(double raw_steer_dot, double dt);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ParamBank param_;
    EKF ekf_;

    ros::Subscriber imu_sub_;
    ros::Subscriber wheel_speed_sub_;
    ros::Subscriber steer_sub_;
    ros::Subscriber torque_sub_;
    ros::Subscriber ins_sub_;

    ros::Publisher state_pub_;
    ros::Publisher odometry_pub_;
    ros::Publisher odometry_debug_pub_;
    tf2_ros::TransformBroadcaster tf_br_;

    ros::Timer loop_timer_;
    ros::Timer publish_timer_;

    double loop_hz_ = 0.0;
    double gravity_init_duration_s_ = 0.0;

    bool gravity_calibration_started_ = false;
    bool gravity_calibration_done_ = false;
    ros::Time gravity_calibration_start_time_;
    double gravity_ax_sum_ = 0.0;
    double gravity_ay_sum_ = 0.0;
    std::size_t gravity_sample_count_ = 0;

    bool new_imu_sample_ = false;
    bool new_wheel_speed_sample_ = false;

    bool new_steer_sample_ = false;
    bool new_torque_sample_ = false;

    bool has_prev_steer_sample_ = false;
    double prev_steer_ = 0.0;
    ros::Time prev_steer_stamp_;
    std::vector<double> steer_dot_filter_states_;

    nav_msgs::Odometry ins_odom_;
    bool has_ins_odom_ = false;
    int n_samples_ = 0;
    double mse_error_pos_ = 0.0;
    double mse_error_yaw_ = 0.0;
    double mse_error_vx_ = 0.0;
    double mse_error_vy_ = 0.0;
    double mse_error_yaw_rate_ = 0.0;
};

} // namespace kalman_filter_kinematic