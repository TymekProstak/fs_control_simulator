#include "EKF/wrapper.hpp"
#include <stdexcept>
#include <cmath>

#include <tf2/utils.h>

namespace kalman_filter_kinematic
{

EKFWrapper::EKFWrapper(ros::NodeHandle& nh,
                       ros::NodeHandle& pnh,
                       const ParamBank& param_bank)
    : nh_(nh)
    , pnh_(pnh)
    , param_(param_bank)
    , ekf_(param_)
{
    const std::string imu_topic            = getRequiredStringParam("topics/imu");
    const std::string wheel_speed_topic    = getRequiredStringParam("topics/wheel_speed_vx");
    const std::string steer_topic          = getRequiredStringParam("topics/steer");
    const std::string torque_topic         = getRequiredStringParam("topics/torque_cmd");
    const std::string ins_pose_topic       = getRequiredStringParam("topics/ins_pose");
    const std::string state_topic          = getRequiredStringParam("topics/state_out");
    const std::string odometry_topic       = getRequiredStringParam("topics/odometry_out");
    const std::string odometry_debug_topic = getRequiredStringParam("topics/odometry_debug");

    loop_hz_ = getRequiredDoubleParam("loop_hz");
    gravity_init_duration_s_ = getRequiredDoubleParam("gravity_init_duration_s");

    constexpr double publish_hz = 50.0;

    if (loop_hz_ <= 0.0) {
        throw std::runtime_error("EKFWrapper: loop_hz must be > 0");
    }

    if (gravity_init_duration_s_ <= 0.0) {
        throw std::runtime_error("EKFWrapper: gravity_init_duration_s must be > 0");
    }

    const int steer_dot_filter_order =
        static_cast<int>(std::lround(param_.get("kalman_filter_kinematic.steer_dot_filter.order")));

    if (steer_dot_filter_order < 0) {
        throw std::runtime_error("EKFWrapper: steer_dot_filter.order must be >= 0");
    }

    steer_dot_filter_states_.assign(static_cast<std::size_t>(steer_dot_filter_order), 0.0);

    const bool gravity_comp_enabled =
        param_.get("kalman_filter_kinematic.logic.enable_gravity_compensation") > 0.5;

    if (!gravity_comp_enabled) {
        gravity_calibration_started_ = true;
        gravity_calibration_done_ = true;
    }

    imu_sub_ = nh_.subscribe(imu_topic, 1,
                             &EKFWrapper::imuCallback, this);

    wheel_speed_sub_ = nh_.subscribe(wheel_speed_topic, 1,
                                     &EKFWrapper::wheelSpeedCallback, this);

    steer_sub_ = nh_.subscribe(steer_topic, 1,
                               &EKFWrapper::steerCallback, this);

    torque_sub_ = nh_.subscribe(torque_topic, 1,
                                &EKFWrapper::torqueCallback, this);

    ins_sub_ = nh_.subscribe(ins_pose_topic, 1,
                             &EKFWrapper::InsCallback, this);

    state_pub_ = nh_.advertise<dv_interfaces::Odom_state>(state_topic, 1);
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(odometry_topic, 1);
    odometry_debug_pub_ = nh_.advertise<dv_interfaces::OdomDebug>(odometry_debug_topic, 1);

    loop_timer_ = nh_.createTimer(
        ros::Duration(1.0 / loop_hz_),
        &EKFWrapper::loopTimerCallback,
        this
    );

    publish_timer_ = nh_.createTimer(
        ros::Duration(1.0 / publish_hz),
        &EKFWrapper::publishTimerCallback,
        this
    );
}

void EKFWrapper::spin()
{
    ros::spin();
}

std::string EKFWrapper::getRequiredStringParam(const std::string& name) const
{
    std::string value;
    if (!pnh_.getParam(name, value)) {
        throw std::runtime_error("EKFWrapper: missing required ROS param '" + name + "'");
    }
    return value;
}

double EKFWrapper::getRequiredDoubleParam(const std::string& name) const
{
    double value = 0.0;
    if (!pnh_.getParam(name, value)) {
        throw std::runtime_error("EKFWrapper: missing required ROS param '" + name + "'");
    }
    return value;
}

int EKFWrapper::getRequiredIntParam(const std::string& name) const
{
    int value = 0;
    if (!pnh_.getParam(name, value)) {
        throw std::runtime_error("EKFWrapper: missing required ROS param '" + name + "'");
    }
    return value;
}

void EKFWrapper::imuCallback(const dv_interfaces::Imu::ConstPtr& msg_ptr)
{
    const double ax = msg_ptr->acc.x;
    const double ay = msg_ptr->acc.y;
    const double r  = msg_ptr->gyro.z;

    ekf_.setImuMeasurement(ax, ay, r);
    new_imu_sample_ = true;

    const bool gravity_comp_enabled =
        param_.get("kalman_filter_kinematic.logic.enable_gravity_compensation") > 0.5;

    if (!gravity_comp_enabled) {
        return;
    }

    const ros::Time stamp = msg_ptr->header.stamp.isZero() ? ros::Time::now() : msg_ptr->header.stamp;

    if (!gravity_calibration_started_) {
        gravity_calibration_started_ = true;
        gravity_calibration_start_time_ = stamp;
    }

    if (!gravity_calibration_done_) {
        gravity_ax_sum_ += ax;
        gravity_ay_sum_ += ay;
        ++gravity_sample_count_;

        const double elapsed = (stamp - gravity_calibration_start_time_).toSec();
        if (elapsed >= gravity_init_duration_s_) {
            finalizeGravityCalibration();
        }
    }
}

void EKFWrapper::wheelSpeedCallback(const dv_interfaces::DV_board::ConstPtr& msg_ptr)
{
    ekf_.setWheelSpeedMeasurement(msg_ptr->odom.velocity);
    new_wheel_speed_sample_ = true;
}

void EKFWrapper::steerCallback(const dv_interfaces::Steer::ConstPtr& msg_ptr)
{
    const ros::Time now = ros::Time::now();
    const double steer = msg_ptr->steer;
    const double steer_dot = updateAndFilterSteerDot(steer, now);

    ekf_.setSteeringMeasurement(steer, steer_dot);
    new_steer_sample_ = true;
}

void EKFWrapper::torqueCallback(const dv_interfaces::Control::ConstPtr& msg_ptr)
{
    ekf_.setTorqueCommand((msg_ptr->movement) / 100.0 * 466.25);
    new_torque_sample_ = true;
}

void EKFWrapper::InsCallback(const nav_msgs::Odometry::ConstPtr& msg_ptr)
{
    const nav_msgs::Odometry& msg = *msg_ptr;

    ins_odom_ = msg;
    has_ins_odom_ = true;

    dv_interfaces::OdomDebug debug_msg;
    debug_msg.header = msg.header;

    const State& s = ekf_.getState();

    const double error_x = s.x - msg.pose.pose.position.x;
    const double error_y = s.y - msg.pose.pose.position.y;

    const double ins_yaw = tf2::getYaw(msg.pose.pose.orientation);
    const double yaw_diff = s.yaw - ins_yaw;
    const double error_yaw = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));

    const double vx_world = msg.twist.twist.linear.x;
    const double vy_world = msg.twist.twist.linear.y;
    //  const double vy_world = -1* msg.twist.twist.linear.y;
    const double c = std::cos(ins_yaw);
    const double ss = std::sin(ins_yaw);

    const double vx_body =  c * vx_world + ss * vy_world;
    const double vy_body = -ss * vx_world + c * vy_world;

    const double error_vx = s.vx - vx_body;
    const double error_vy = s.vy - vy_body;
    const double error_yaw_rate = s.yaw_rate - msg.twist.twist.angular.z;

    ++n_samples_;

    mse_error_pos_ += error_x * error_x + error_y * error_y;
    mse_error_yaw_ += error_yaw * error_yaw;
    mse_error_vx_ += error_vx * error_vx;
    mse_error_vy_ += error_vy * error_vy;
    mse_error_yaw_rate_ += error_yaw_rate * error_yaw_rate;

    debug_msg.error_x = error_x;
    debug_msg.error_y = error_y;
    debug_msg.error_yaw = error_yaw;
    debug_msg.error_vx = error_vx;
    debug_msg.error_vy = error_vy;
    debug_msg.error_yaw_rate = error_yaw_rate;

    debug_msg.error_pos_avg = std::sqrt(mse_error_pos_ / static_cast<double>(n_samples_));
    debug_msg.error_vx_avg = std::sqrt(mse_error_vx_ / static_cast<double>(n_samples_));
    debug_msg.error_vy_avg = std::sqrt(mse_error_vy_ / static_cast<double>(n_samples_));
    debug_msg.error_yaw_avg = std::sqrt(mse_error_yaw_ / static_cast<double>(n_samples_));
    debug_msg.error_yaw_rate_avg = std::sqrt(mse_error_yaw_rate_ / static_cast<double>(n_samples_));

    odometry_debug_pub_.publish(debug_msg);
}

void EKFWrapper::loopTimerCallback(const ros::TimerEvent& event)
{
    const bool gravity_comp_enabled =
        param_.get("kalman_filter_kinematic.logic.enable_gravity_compensation") > 0.5;

    if (gravity_comp_enabled && !gravity_calibration_done_) {
        return;
    }

    const double dt = (event.current_real - event.last_real).toSec();
    if (dt <= 0.0) {
        return;
    }

    ekf_.predict(dt);

    if (new_imu_sample_) {
        ekf_.updateFromImu();
        new_imu_sample_ = false;
    }

    if (new_wheel_speed_sample_) {
        ekf_.updateFromWheelSpeed();
        new_wheel_speed_sample_ = false;
    }

    ekf_.updatePseudoKinematicLateralAccel();
    ekf_.updatePseudoTorqueAccel();
    ekf_.updatePseudoZeroLateralVelocity();

    new_steer_sample_ = false;
    new_torque_sample_ = false;

    publishState();
}

void EKFWrapper::publishTimerCallback(const ros::TimerEvent& event)
{
    (void)event;

    const bool gravity_comp_enabled =
        param_.get("kalman_filter_kinematic.logic.enable_gravity_compensation") > 0.5;

    if (gravity_comp_enabled && !gravity_calibration_done_) {
        return;
    }

    publishOdometry();
    publish_odom_tf();
}

void EKFWrapper::finalizeGravityCalibration()
{
    if (gravity_sample_count_ == 0) {
        throw std::runtime_error("EKFWrapper: gravity calibration finished with zero samples");
    }

    const double mean_ax = gravity_ax_sum_ / static_cast<double>(gravity_sample_count_);
    const double mean_ay = gravity_ay_sum_ / static_cast<double>(gravity_sample_count_);

    ekf_.setInitialGravityInPlane(mean_ax, mean_ay);

    gravity_calibration_done_ = true;

    ROS_INFO_STREAM("[kalman_filter_kinematic] gravity init done: "
                    << "gx_init=" << mean_ax
                    << ", gy_init=" << mean_ay
                    << ", samples=" << gravity_sample_count_);
}

void EKFWrapper::publishState()
{
    const State& s = ekf_.getState();

    dv_interfaces::Odom_state msg;
    msg.header.stamp = ros::Time::now();
    msg.x = s.x;
    msg.y = s.y;
    msg.yaw = s.yaw;
    msg.vx = s.vx;
    msg.vy = s.vy;
    msg.yaw_rate = s.yaw_rate;
    msg.ax = s.ax;
    msg.ay = s.ay;
    msg.steer = s.steer;
    msg.steer_dot = s.steer_dot;
    msg.T = s.T;
    msg.imu_ax_raw = s.imu_ax_raw;
    msg.imu_ay_raw = s.imu_ay_raw;
    msg.imu_yaw_rate_raw = s.imu_yaw_rate_raw;
    msg.wheel_speed_vx = s.wheel_speed_vx;
    msg.g_x_init = s.g_x_init;
    msg.g_y_init = s.g_y_init;
    msg.g_in_plane_modul_init = s.g_in_plane_modul_init;

    state_pub_.publish(msg);
}

void EKFWrapper::publishOdometry()
{
    const State& s = ekf_.getState();

    const double c = std::cos(s.yaw);
    const double ss = std::sin(s.yaw);

    const double vx_map = c * s.vx - ss * s.vy;
    const double vy_map = ss * s.vx + c * s.vy;

    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.child_frame_id = "map";

    msg.pose.pose.position.x = s.x;
    msg.pose.pose.position.y = s.y;
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, s.yaw);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.twist.twist.linear.x = vx_map;
    msg.twist.twist.linear.y = vy_map;
    msg.twist.twist.linear.z = 0.0;
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = s.yaw_rate;

    odometry_pub_.publish(msg);
}

void EKFWrapper::publish_odom_tf()
{
    const State& s = ekf_.getState();

    geometry_msgs::TransformStamped tf_ekf;
    tf_ekf.header.stamp = ros::Time::now();
    tf_ekf.header.frame_id = "map";
    tf_ekf.child_frame_id  = "bolide_odom_CoG";

    tf_ekf.transform.translation.x = s.x;
    tf_ekf.transform.translation.y = s.y;
    tf_ekf.transform.translation.z = 0.0;

    tf2::Quaternion q2;
    q2.setRPY(0.0, 0.0, s.yaw);
    tf_ekf.transform.rotation.x = q2.x();
    tf_ekf.transform.rotation.y = q2.y();
    tf_ekf.transform.rotation.z = q2.z();
    tf_ekf.transform.rotation.w = q2.w();

    tf_br_.sendTransform(tf_ekf);
}

double EKFWrapper::updateAndFilterSteerDot(double steer_now, const ros::Time& stamp)
{
    if (!has_prev_steer_sample_) {
        has_prev_steer_sample_ = true;
        prev_steer_ = steer_now;
        prev_steer_stamp_ = stamp;
        return 0.0;
    }

    const double dt = (stamp - prev_steer_stamp_).toSec();

    double raw_steer_dot = 0.0;
    if (dt > 1e-6) {
        raw_steer_dot = (steer_now - prev_steer_) / dt;
    }

    prev_steer_ = steer_now;
    prev_steer_stamp_ = stamp;

    return applySteerDotFilterCascade(raw_steer_dot, dt);
}

double EKFWrapper::applySteerDotFilterCascade(double raw_steer_dot, double dt)
{
    if (dt <= 0.0) {
        return raw_steer_dot;
    }

    const double cutoff_hz =
        param_.get("kalman_filter_kinematic.steer_dot_filter.cutoff_hz");

    const int order =
        static_cast<int>(std::lround(param_.get("kalman_filter_kinematic.steer_dot_filter.order")));

    if (order <= 0 || cutoff_hz <= 0.0) {
        return raw_steer_dot;
    }

    const double alpha = std::exp(-2.0 * M_PI * cutoff_hz * dt);

    double value = raw_steer_dot;
    for (int i = 0; i < order; ++i) {
        steer_dot_filter_states_[static_cast<std::size_t>(i)] =
            alpha * steer_dot_filter_states_[static_cast<std::size_t>(i)] +
            (1.0 - alpha) * value;

        value = steer_dot_filter_states_[static_cast<std::size_t>(i)];
    }

    return value;
}

} // namespace kalman_filter_kinematic