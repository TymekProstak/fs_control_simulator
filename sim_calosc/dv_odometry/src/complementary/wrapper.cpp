#include "complementary/wrapper.hpp"

#include <stdexcept>
#include <cmath>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

namespace complementary_filter_kinematic
{

ComplementaryFilterWrapper::ComplementaryFilterWrapper(ros::NodeHandle& nh,
                                                       ros::NodeHandle& pnh,
                                                       const ParamBank& param_bank)
    : nh_(nh)
    , pnh_(pnh)
    , param_(param_bank)
    , filter_(param_)
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
        throw std::runtime_error("ComplementaryFilterWrapper: loop_hz must be > 0");
    }

    if (gravity_init_duration_s_ <= 0.0) {
        throw std::runtime_error("ComplementaryFilterWrapper: gravity_init_duration_s must be > 0");
    }

    const bool gravity_comp_enabled =
        param_.get("enable_gravity_compensation") > 0.5;

    if (!gravity_comp_enabled) {
        gravity_calibration_started_ = true;
        gravity_calibration_done_ = true;
    }

    imu_sub_ = nh_.subscribe(imu_topic, 1,
                             &ComplementaryFilterWrapper::imuCallback, this);

    wheel_speed_sub_ = nh_.subscribe(wheel_speed_topic, 1,
                                     &ComplementaryFilterWrapper::wheelSpeedCallback, this);

    steer_sub_ = nh_.subscribe(steer_topic, 1,
                               &ComplementaryFilterWrapper::steerCallback, this);

    torque_sub_ = nh_.subscribe(torque_topic, 1,
                                &ComplementaryFilterWrapper::torqueCallback, this);

    ins_sub_ = nh_.subscribe(ins_pose_topic, 1,
                             &ComplementaryFilterWrapper::InsCallback, this);

    state_pub_ = nh_.advertise<dv_interfaces::Odom_state>(state_topic, 1);
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(odometry_topic, 1);
    odometry_debug_pub_ = nh_.advertise<dv_interfaces::OdomDebug>(odometry_debug_topic, 1);

    loop_timer_ = nh_.createTimer(
        ros::Duration(1.0 / loop_hz_),
        &ComplementaryFilterWrapper::loopTimerCallback,
        this
    );

    publish_timer_ = nh_.createTimer(
        ros::Duration(1.0 / publish_hz),
        &ComplementaryFilterWrapper::publishTimerCallback,
        this
    );
}

void ComplementaryFilterWrapper::spin()
{
    ros::spin();
}

std::string ComplementaryFilterWrapper::getRequiredStringParam(const std::string& name) const
{
    std::string value;
    if (!pnh_.getParam(name, value)) {
        throw std::runtime_error("ComplementaryFilterWrapper: missing required ROS param '" + name + "'");
    }
    return value;
}

double ComplementaryFilterWrapper::getRequiredDoubleParam(const std::string& name) const
{
    double value = 0.0;
    if (!pnh_.getParam(name, value)) {
        throw std::runtime_error("ComplementaryFilterWrapper: missing required ROS param '" + name + "'");
    }
    return value;
}

int ComplementaryFilterWrapper::getRequiredIntParam(const std::string& name) const
{
    int value = 0;
    if (!pnh_.getParam(name, value)) {
        throw std::runtime_error("ComplementaryFilterWrapper: missing required ROS param '" + name + "'");
    }
    return value;
}

void ComplementaryFilterWrapper::imuCallback(const dv_interfaces::Imu::ConstPtr& msg_ptr)
{
    const double ax = msg_ptr->acc.x;
    const double ay = msg_ptr->acc.y;
    const double r  = msg_ptr->gyro.z;

    filter_.setImuMeasurement(ax, ay, r);
    new_imu_sample_ = true;

    const bool gravity_comp_enabled =
        param_.get("enable_gravity_compensation") > 0.5;

    if (!gravity_comp_enabled) {
        return;
    }

    const ros::Time stamp =
        msg_ptr->header.stamp.isZero() ? ros::Time::now() : msg_ptr->header.stamp;

    if (!gravity_calibration_started_) {
        gravity_calibration_started_ = true;
        gravity_calibration_start_time_ = stamp;
    }

    if (!gravity_calibration_done_) {
        gravity_ax_sum_ += ax;
        gravity_ay_sum_ += ay;
        ++gravity_sample_count_;



    if(!gyro_bias_calibration_started_) {
        gyro_bias_calibration_started_ = true;
        gyro_bias_calibration_start_time_ = stamp;
    }

    if(!gyro_bias_calibration_done_) {
        gyro_bias_sum_ += r;
        ++gyro_bias_sample_count_;
    }


        const double elapsed = (stamp - gravity_calibration_start_time_).toSec();
        if (elapsed >= gravity_init_duration_s_) {
            finalizeGravityCalibration();
        }
        const double gyro_bias_calibration_elapsed = (stamp - gyro_bias_calibration_start_time_).toSec();
        if(gyro_bias_calibration_elapsed >= gravity_init_duration_s_) {
            finalizeBiasCalibration();
        }
    }
}

void ComplementaryFilterWrapper::wheelSpeedCallback(const dv_interfaces::DV_board::ConstPtr& msg_ptr)
{
    filter_.setWheelSpeedMeasurement(msg_ptr->odom.velocity);
    new_wheel_speed_sample_ = true;
}

void ComplementaryFilterWrapper::steerCallback(const dv_interfaces::Steer::ConstPtr& msg_ptr)
{
    // steer_dot liczy już sam filtr komplementarny z filtrowanego steer
    filter_.setSteeringMeasurement(msg_ptr->steer, 0.0);
}

void ComplementaryFilterWrapper::torqueCallback(const dv_interfaces::Control::ConstPtr& msg_ptr)
{
    filter_.setTorqueCommand((msg_ptr->movement) / 100.0 * 466.25);
}

void ComplementaryFilterWrapper::InsCallback(const nav_msgs::Odometry::ConstPtr& msg_ptr)
{
    const nav_msgs::Odometry& msg = *msg_ptr;

    ins_odom_ = msg;
    has_ins_odom_ = true;

    const double ins_yaw = tf2::getYaw(msg.pose.pose.orientation);
   

    dv_interfaces::OdomDebug debug_msg;
    debug_msg.header = msg.header;

    const State& s = filter_.getState();

    const double error_x = s.x - msg.pose.pose.position.x;
    const double error_y = s.y - msg.pose.pose.position.y;

    const double yaw_diff = s.yaw - ins_yaw;
    const double error_yaw = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));

    const double vx_world = msg.twist.twist.linear.x;
    const double vy_world = msg.twist.twist.linear.y;

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

     filter_.setInsPose(msg.pose.pose.position.x,
                       msg.pose.pose.position.y,
                       ins_yaw);

    odometry_debug_pub_.publish(debug_msg);
}

void ComplementaryFilterWrapper::loopTimerCallback(const ros::TimerEvent& event)
{
    const bool gravity_comp_enabled =
        param_.get("enable_gravity_compensation") > 0.5;

    if (gravity_comp_enabled && !gravity_calibration_done_) {
        return;
    }

    const double dt = (event.current_real - event.last_real).toSec();
    if (dt <= 0.0) {
        return;
    }

    if (new_imu_sample_) {
        filter_.updateFromImu(dt);
        new_imu_sample_ = false;
    }

    if (new_wheel_speed_sample_) {
        filter_.updateFromWheelSpeed();
        new_wheel_speed_sample_ = false;
    }

    publishState();
}

void ComplementaryFilterWrapper::publishTimerCallback(const ros::TimerEvent& event)
{
    (void)event;

    const bool gravity_comp_enabled =
        param_.get("enable_gravity_compensation") > 0.5;

    if (gravity_comp_enabled && !gravity_calibration_done_) {
        return;
    }

    publishOdometry();
    publish_odom_tf();
}

void ComplementaryFilterWrapper::finalizeGravityCalibration()
{
    if (gravity_sample_count_ == 0) {
        throw std::runtime_error("ComplementaryFilterWrapper: gravity calibration finished with zero samples");
    }

    const double mean_ax = gravity_ax_sum_ / static_cast<double>(gravity_sample_count_);
    const double mean_ay = gravity_ay_sum_ / static_cast<double>(gravity_sample_count_);

    filter_.setInitialGravityInPlane(mean_ax, mean_ay);

    gravity_calibration_done_ = true;

    ROS_INFO_STREAM("[complementary_filter_kinematic] gravity init done: "
                    << "gx_init=" << mean_ax
                    << ", gy_init=" << mean_ay
                    << ", samples=" << gravity_sample_count_);
}


void ComplementaryFilterWrapper::finalizeBiasCalibration()
{
    if (gyro_bias_sample_count_ == 0) {
        throw std::runtime_error("ComplementaryFilterWrapper: gyro bias calibration finished with zero samples");
    }

    const double mean_gyro_bias = gyro_bias_sum_ / static_cast<double>(gyro_bias_sample_count_);

    filter_.setInitialGyroBias(mean_gyro_bias);

    gyro_bias_calibration_done_ = true;

    ROS_INFO_STREAM("[complementary_filter_kinematic] gyro bias init done: "
                    << "gyro_bias=" << mean_gyro_bias
                    << ", samples=" << gyro_bias_sample_count_);
}

void ComplementaryFilterWrapper::publishState()
{
    const State& s = filter_.getState();
    const RawInputs& raw = filter_.getRawInputs();
    const ProcessedSignals& proc = filter_.getProcessedSignals();

    dv_interfaces::Odom_state msg;
    msg.header.stamp = ros::Time::now();

    msg.x = s.x;
    msg.y = s.y;
    msg.yaw = s.yaw;
    msg.vx = s.vx;
    msg.vy = s.vy;
    msg.yaw_rate = s.yaw_rate;

    // w filtrze komplementarnym publikujemy bieżące sygnały używane przez filtr
    msg.ax = proc.imu_ax_comp;
    msg.ay = proc.imu_ay_comp;
    msg.steer = proc.steer_filtered;
    msg.steer_dot = proc.steer_dot_filtered;
    msg.T = raw.torque_cmd;

    msg.imu_ax_raw = raw.imu_ax_raw;
    msg.imu_ay_raw = raw.imu_ay_raw;
    msg.imu_yaw_rate_raw = raw.imu_yaw_rate_raw;
    msg.wheel_speed_vx = raw.wheel_speed_vx;

    msg.g_x_init = s.g_x_init;
    msg.g_y_init = s.g_y_init;
    msg.g_in_plane_modul_init = s.g_in_plane_modul_init;

    msg.discrepancy_r_kin_gyro = proc.r_kin_gyro_discrepancy;

    state_pub_.publish(msg);
}

void ComplementaryFilterWrapper::publishOdometry()
{
    const State& s = filter_.getState();

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

void ComplementaryFilterWrapper::publish_odom_tf()
{
    const State& s = filter_.getState();

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id  = "bolide_odom_CoG";

    tf_msg.transform.translation.x = s.x;
    tf_msg.transform.translation.y = s.y;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, s.yaw);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_br_.sendTransform(tf_msg);
}

} // namespace complementary_filter_kinematic