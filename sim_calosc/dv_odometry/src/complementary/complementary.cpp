#include "complementary/complementary.hpp"

#include <algorithm>

namespace complementary_filter_kinematic
{

//// ============================================================
////  TimeAveragingWindow
//// ============================================================
TimeAveragingWindow::TimeAveragingWindow(double window_time_s)
    : window_time_s_(window_time_s)
{
}

void TimeAveragingWindow::setWindowTime(double window_time_s)
{
    window_time_s_ = std::max(0.0, window_time_s);
    trimToWindow();
}

void TimeAveragingWindow::reset()
{
    weighted_sum_ = 0.0;
    total_time_ = 0.0;
    buffer_.clear();
}

void TimeAveragingWindow::push(double value, double dt)
{
    if (dt <= 0.0) {
        return;
    }

    buffer_.push_back({dt, value});
    weighted_sum_ += value * dt;
    total_time_ += dt;

    trimToWindow();
}

double TimeAveragingWindow::mean() const
{
    if (total_time_ <= 1e-12) {
        return 0.0;
    }
    return weighted_sum_ / total_time_;
}

bool TimeAveragingWindow::empty() const
{
    return buffer_.empty();
}

double TimeAveragingWindow::totalTime() const
{
    return total_time_;
}

void TimeAveragingWindow::trimToWindow()
{
    if (window_time_s_ <= 0.0) {
        reset();
        return;
    }

    while (!buffer_.empty() && total_time_ > window_time_s_) {
        const double excess = total_time_ - window_time_s_;
        auto& front = buffer_.front();

        if (front.first <= excess + 1e-12) {
            weighted_sum_ -= front.second * front.first;
            total_time_ -= front.first;
            buffer_.pop_front();
        } else {
            weighted_sum_ -= front.second * excess;
            total_time_ -= excess;
            front.first -= excess;
            break;
        }
    }
}

//// ============================================================
////  OutlierRejector
//// ============================================================
void OutlierRejector::configure(int history_size_samples,
                                double relative_deviation_threshold,
                                double relative_deviation_ref_epsilon,
                                double absolute_reject_threshold,
                                int max_consecutive_rejects)
{
    history_size_samples_ = std::max(1, history_size_samples);
    relative_deviation_threshold_ = std::max(0.0, relative_deviation_threshold);
    relative_deviation_ref_epsilon_ = std::max(1e-9, relative_deviation_ref_epsilon);
    absolute_reject_threshold_ = std::max(0.0, absolute_reject_threshold);
    max_consecutive_rejects_ = std::max(0, max_consecutive_rejects);

    reset();
}

void OutlierRejector::reset()
{
    initialized_ = false;
    consecutive_rejects_ = 0;
    last_accepted_ = 0.0;
    history_.clear();
}

bool OutlierRejector::initialized() const
{
    return initialized_;
}

double OutlierRejector::referenceMean() const
{
    if (history_.empty()) {
        return 0.0;
    }

    double sum = 0.0;
    for (double v : history_) {
        sum += v;
    }
    return sum / static_cast<double>(history_.size());
}

void OutlierRejector::pushAccepted(double sample)
{
    history_.push_back(sample);
    while (static_cast<int>(history_.size()) > history_size_samples_) {
        history_.pop_front();
    }
    last_accepted_ = sample;
    initialized_ = true;
}

double OutlierRejector::filter(double sample)
{
    if (!initialized_) {
        pushAccepted(sample);
        consecutive_rejects_ = 0;
        return sample;
    }

    const double ref = referenceMean();
    const double denom = std::max(std::abs(ref), relative_deviation_ref_epsilon_);
    const double rel_dev = std::abs(sample - ref) / denom;

    const bool reject_abs = std::abs(sample) > absolute_reject_threshold_;
    const bool reject_rel = rel_dev > relative_deviation_threshold_;
    const bool reject = reject_abs || reject_rel;

    if (reject && consecutive_rejects_ < max_consecutive_rejects_) {
        ++consecutive_rejects_;
        return last_accepted_;
    }

    consecutive_rejects_ = 0;
    pushAccepted(sample);
    return sample;
}

//// ============================================================
////  complementary_filter
//// ============================================================
complementary_filter::complementary_filter(const ParamBank& param_bank)
    : param_(param_bank),
      ax_abs_window_(readDoubleParam("ax_time")),
      ay_abs_window_(readDoubleParam("ay_time")),
      torque_cmd_abs_window_(readDoubleParam("torque_cmd_time")),
      steer_dot_abs_window_(readDoubleParam("steer_dot_time")),
      r_discrepancy_abs_window_(readDoubleParam("r_kin_gyro_discrepancy_time")),

      standstill_wheel_speed_abs_window_(readDoubleParam("standstill_time")),
      standstill_ax_abs_window_(readDoubleParam("standstill_time")),
      standstill_ay_abs_window_(readDoubleParam("standstill_time")),
      standstill_yaw_rate_abs_window_(readDoubleParam("standstill_time")),
      standstill_steer_abs_window_(readDoubleParam("standstill_time")),
      standstill_steer_dot_abs_window_(readDoubleParam("standstill_time")),
      standstill_torque_abs_window_(readDoubleParam("standstill_time")),

      straight_ay_abs_window_(readDoubleParam("straight_line_driving_time")),
      straight_yaw_rate_abs_window_(readDoubleParam("straight_line_driving_time")),
      straight_steer_abs_window_(readDoubleParam("straight_line_driving_time")),
      straight_steer_dot_abs_window_(readDoubleParam("straight_line_driving_time"))
{
    imu_ax_rejector_.configure(
        readIntParam("imu_ax_history_size_samples"),
        readDoubleParam("imu_ax_relative_deviation_threshold"),
        readDoubleParam("imu_ax_relative_deviation_ref_epsilon"),
        readDoubleParam("imu_ax_absolute_reject_threshold"),
        readIntParam("imu_ax_max_consecutive_rejects"));

    imu_ay_rejector_.configure(
        readIntParam("imu_ay_history_size_samples"),
        readDoubleParam("imu_ay_relative_deviation_threshold"),
        readDoubleParam("imu_ay_relative_deviation_ref_epsilon"),
        readDoubleParam("imu_ay_absolute_reject_threshold"),
        readIntParam("imu_ay_max_consecutive_rejects"));

    imu_yaw_rate_rejector_.configure(
        readIntParam("imu_yaw_rate_history_size_samples"),
        readDoubleParam("imu_yaw_rate_relative_deviation_threshold"),
        readDoubleParam("imu_yaw_rate_relative_deviation_ref_epsilon"),
        readDoubleParam("imu_yaw_rate_absolute_reject_threshold"),
        readIntParam("imu_yaw_rate_max_consecutive_rejects"));
}

void complementary_filter::setImuMeasurement(double ax_raw, double ay_raw, double yaw_rate_raw)
{
    raw_.imu_ax_raw = ax_raw;
    raw_.imu_ay_raw = ay_raw;
    raw_.imu_yaw_rate_raw = yaw_rate_raw;
    has_imu_ = true;
}

void complementary_filter::setWheelSpeedMeasurement(double vx_meas)
{
    raw_.wheel_speed_vx = vx_meas;
    has_wheel_speed_ = true;
}

void complementary_filter::setSteeringMeasurement(double steer, double steer_dot)
{
    raw_.steer_raw = steer;
    raw_.steer_dot_raw = steer_dot;
    has_steer_ = true;
}

void complementary_filter::setTorqueCommand(double torque_command)
{
    raw_.torque_cmd = torque_command;
    has_torque_command_ = true;
}

void complementary_filter::setInsPose(double x, double y, double yaw)
{
    raw_.ins_x = x;
    raw_.ins_y = y;
    raw_.ins_yaw = yaw;
    has_ins_ = true;
    if(readBoolParam("enable_ins_fusion")) {
        state_.x = x;
        state_.y = y;
        state_.yaw = yaw;
    }
}

void complementary_filter::setInitialGravityInPlane(double gx_init, double gy_init)
{
    state_.g_x_init = gx_init;
    state_.g_y_init = gy_init;
    state_.g_in_plane_modul_init = std::sqrt(gx_init * gx_init + gy_init * gy_init);
    has_gravity_init_ = true;
}

void complementary_filter::setInitialGyroBias(double gyro_bias)
{
    state_.gyro_bias = gyro_bias;
}

void complementary_filter::updateFromImu(double dt)
{
    if (!has_imu_ || dt <= 0.0) {
        return;
    }

    preprocessImuMeasurements();
    updateGravityCompensatedImu();
    updateFilteredSteering(dt);

    updateKinematicReferences();
    updateHeuristicWindows(dt);
    updateComplementaryGains();

    updateYawRateEstimate();
    propagateLongitudinalVelocity(dt);
    propagateLateralVelocity(dt);

    updateModeWindows(dt);
    evaluateModes();

    applySoftStandstillMode(dt);
    applySoftStraightLineMode(dt);

    if (readBoolParam("clamp_negative_vx_to_zero") && state_.vx < 0.0) {
        state_.vx = 0.0;
    }

    state_.vy = clamp(state_.vy,
                      -readDoubleParam("max_abs_vy"),
                       readDoubleParam("max_abs_vy"));

    state_.yaw_rate = clamp(state_.yaw_rate,
                            -readDoubleParam("max_abs_r"),
                             readDoubleParam("max_abs_r"));

    integratePlanarPose(dt);
}

void complementary_filter::updateFromWheelSpeed()
{
    if (!has_wheel_speed_) {
        return;
    }

    updateLongitudinalVelocityFromWheel();
}

const State& complementary_filter::getState() const
{
    return state_;
}

const RawInputs& complementary_filter::getRawInputs() const
{
    return raw_;
}

const ProcessedSignals& complementary_filter::getProcessedSignals() const
{
    return proc_;
}

const HeuristicSignals& complementary_filter::getHeuristicSignals() const
{
    return heur_;
}

const ModeSignals& complementary_filter::getModeSignals() const
{
    return mode_;
}

const ComplementaryGains& complementary_filter::getGains() const
{
    return gains_;
}

const ModeFlags& complementary_filter::getModeFlags() const
{
    return flags_;
}

void complementary_filter::preprocessImuMeasurements()
{
    proc_.imu_ax_preprocessed =
        imu_ax_rejector_.filter(raw_.imu_ax_raw);

    proc_.imu_ay_preprocessed =
        imu_ay_rejector_.filter(raw_.imu_ay_raw);

    proc_.imu_yaw_rate_preprocessed =
        imu_yaw_rate_rejector_.filter(raw_.imu_yaw_rate_raw) - state_.gyro_bias;
}

void complementary_filter::updateGravityCompensatedImu()
{
    double ax = proc_.imu_ax_preprocessed - state_.ax_bias;
    double ay = proc_.imu_ay_preprocessed - state_.ay_bias;

    if (readBoolParam("enable_gravity_compensation") && has_gravity_init_) {
        ax -= computeGravityBodyX();
        ay -= computeGravityBodyY();
    }

    proc_.imu_ax_comp = ax;
    proc_.imu_ay_comp = ay;
}

void complementary_filter::updateFilteredSteering(double dt)
{
    if (!has_steer_) {
        proc_.steer_filtered = 0.0;
        proc_.steer_dot_filtered = 0.0;
        return;
    }

    proc_.steer_filtered = filterSteerFirstOrder(raw_.steer_raw, dt);
    proc_.steer_dot_filtered = computeSteerDotFromFilteredSteer(proc_.steer_filtered, dt);
}

void complementary_filter::updateKinematicReferences()
{
    proc_.r_kin = computeKinematicYawRate(state_.vx, proc_.steer_filtered);
    proc_.torque_ax_model = computeTorqueBasedLongitudinalAccel(raw_.torque_cmd, state_.vx);
    proc_.r_kin_gyro_discrepancy = std::abs(proc_.r_kin - proc_.imu_yaw_rate_preprocessed);
}

void complementary_filter::updateHeuristicWindows(double dt)
{
    ax_abs_window_.push(std::abs(proc_.imu_ax_comp), dt);
    ay_abs_window_.push(std::abs(proc_.imu_ay_comp), dt);
    torque_cmd_abs_window_.push(std::abs(raw_.torque_cmd), dt);
    steer_dot_abs_window_.push(std::abs(proc_.steer_dot_filtered), dt);
    r_discrepancy_abs_window_.push(std::abs(proc_.r_kin_gyro_discrepancy), dt);

    heur_.ax_abs_mean = ax_abs_window_.mean();
    heur_.ay_abs_mean = ay_abs_window_.mean();
    heur_.torque_cmd_abs_mean = torque_cmd_abs_window_.mean();
    heur_.steer_dot_abs_mean = steer_dot_abs_window_.mean();
    heur_.r_kin_gyro_discrepancy_abs_mean = r_discrepancy_abs_window_.mean();
}

void complementary_filter::updateModeWindows(double dt)
{
    standstill_wheel_speed_abs_window_.push(std::abs(raw_.wheel_speed_vx), dt);
    standstill_ax_abs_window_.push(std::abs(proc_.imu_ax_comp), dt);
    standstill_ay_abs_window_.push(std::abs(proc_.imu_ay_comp), dt);
    standstill_yaw_rate_abs_window_.push(std::abs(state_.yaw_rate), dt);
    standstill_steer_abs_window_.push(std::abs(proc_.steer_filtered), dt);
    standstill_steer_dot_abs_window_.push(std::abs(proc_.steer_dot_filtered), dt);
    standstill_torque_abs_window_.push(std::abs(raw_.torque_cmd), dt);

    straight_ay_abs_window_.push(std::abs(proc_.imu_ay_comp), dt);
    straight_yaw_rate_abs_window_.push(std::abs(state_.yaw_rate), dt);
    straight_steer_abs_window_.push(std::abs(proc_.steer_filtered), dt);
    straight_steer_dot_abs_window_.push(std::abs(proc_.steer_dot_filtered), dt);

    mode_.wheel_speed_abs_mean = standstill_wheel_speed_abs_window_.mean();
    mode_.ax_abs_mean = standstill_ax_abs_window_.mean();
    mode_.ay_abs_mean = standstill_ay_abs_window_.mean();
    mode_.yaw_rate_abs_mean = standstill_yaw_rate_abs_window_.mean();
    mode_.steer_abs_mean = standstill_steer_abs_window_.mean();
    mode_.steer_dot_abs_mean = standstill_steer_dot_abs_window_.mean();
    mode_.torque_cmd_abs_mean = standstill_torque_abs_window_.mean();
}

void complementary_filter::updateComplementaryGains()
{
    const double c_r_steer_dot =
        softHardConfidence(heur_.steer_dot_abs_mean,
                           readDoubleParam("r_steer_dot_soft_threshold"),
                           readDoubleParam("r_steer_dot_hard_threshold"));

    const double c_r_ay =
        softHardConfidence(heur_.ay_abs_mean,
                           readDoubleParam("r_ay_soft_threshold"),
                           readDoubleParam("r_ay_hard_threshold"));

    const double c_r_disc =
        softHardConfidence(heur_.r_kin_gyro_discrepancy_abs_mean,
                           readDoubleParam("r_discrepancy_soft_threshold"),
                           readDoubleParam("r_discrepancy_hard_threshold"));

    const double gate_r_vx =
        (state_.vx >= readDoubleParam("r_min_vx_for_kinematic_correction")) ? 1.0 : 0.0;

    const double gate_r_steer =
        (std::abs(proc_.steer_filtered) >= readDoubleParam("r_min_abs_steer_for_kinematic_correction")) ? 1.0 : 0.0;

    const double c_r_geom =
        std::cbrt(std::max(0.0, c_r_steer_dot * c_r_ay * c_r_disc));

    const double c_r = gate_r_vx * gate_r_steer * c_r_geom;

    double w_kin =
        lerp(readDoubleParam("r_kinematic_weight_min"),
             readDoubleParam("r_kinematic_weight_max"),
             c_r);

    double w_gyro = 1.0 - w_kin;

    if (!readBoolParam("enable_r_kinematic_correction")) {
        w_gyro = 1.0;
        w_kin = 0.0;
    }

    gains_.r_gyro_weight = w_gyro;
    gains_.r_kinematic_weight = w_kin;

    const double c_vx_ax =
        softHardConfidence(heur_.ax_abs_mean,
                           readDoubleParam("vx_ax_activity_soft_threshold"),
                           readDoubleParam("vx_ax_activity_hard_threshold"));

    const double c_vx_torque =
        softHardConfidence(heur_.torque_cmd_abs_mean,
                           readDoubleParam("vx_torque_cmd_soft_threshold"),
                           readDoubleParam("vx_torque_cmd_hard_threshold"));

    const double gate_vx =
        (std::max(std::abs(state_.vx), std::abs(raw_.wheel_speed_vx))
            >= readDoubleParam("vx_min_vx_for_wheel_update")) ? 1.0 : 0.0;

    const double c_vx_geom =
        std::sqrt(std::max(0.0, c_vx_ax * c_vx_torque));

    const double c_vx = gate_vx * c_vx_geom;

    gains_.vx_wheel_weight =
        lerp(readDoubleParam("vx_wheel_weight_min"),
             readDoubleParam("vx_wheel_weight_max"),
             c_vx);

    const double c_vy_ay =
        softHardConfidence(heur_.ay_abs_mean,
                           readDoubleParam("vy_ay_activity_soft_threshold"),
                           readDoubleParam("vy_ay_activity_hard_threshold"));

    const double c_vy_steer_dot =
        softHardConfidence(heur_.steer_dot_abs_mean,
                           readDoubleParam("vy_steer_dot_soft_threshold"),
                           readDoubleParam("vy_steer_dot_hard_threshold"));

    const double c_vy_disc =
        softHardConfidence(heur_.r_kin_gyro_discrepancy_abs_mean,
                           readDoubleParam("vy_r_kin_gyro_discrepancy_soft_threshold"),
                           readDoubleParam("vy_r_kin_gyro_discrepancy_hard_threshold"));

    const double gate_vy =
        (state_.vx >= readDoubleParam("vy_min_vx_for_kinematic_vy_reference")) ? 1.0 : 0.0;

    const double c_vy_geom =
        std::cbrt(std::max(0.0, c_vy_ay * c_vy_steer_dot * c_vy_disc));

    const double c_vy = gate_vy * c_vy_geom;

    gains_.vy_kinematic_weight =
        lerp(readDoubleParam("vy_kinematic_weight_min"),
             readDoubleParam("vy_kinematic_weight_max"),
             c_vy);
}

void complementary_filter::updateYawRateEstimate()
{
    const double r_gyro = proc_.imu_yaw_rate_preprocessed;
    state_.yaw_rate =
        gains_.r_gyro_weight * r_gyro +
        gains_.r_kinematic_weight * proc_.r_kin;
}

void complementary_filter::propagateLongitudinalVelocity(double dt)
{
    const double vx_dot = proc_.imu_ax_comp + state_.yaw_rate * state_.vy;
    state_.vx += vx_dot * dt;
}

void complementary_filter::propagateLateralVelocity(double dt)
{
    const double vy_pred = state_.vy + (proc_.imu_ay_comp - state_.yaw_rate * state_.vx) * dt;

    proc_.vy_kin = computeKinematicVyFromRearAxleZeroSlip(state_.yaw_rate);

    state_.vy =
        (1.0 - gains_.vy_kinematic_weight) * vy_pred +
        gains_.vy_kinematic_weight * proc_.vy_kin;
}

void complementary_filter::updateLongitudinalVelocityFromWheel()
{
    state_.vx =
        (1.0 - gains_.vx_wheel_weight) * state_.vx +
        gains_.vx_wheel_weight * raw_.wheel_speed_vx;

    if (readBoolParam("clamp_negative_vx_to_zero") && state_.vx < 0.0) {
        state_.vx = 0.0;
    }
}

void complementary_filter::evaluateModes()
{
    flags_.standstill =
        (mode_.wheel_speed_abs_mean < readDoubleParam("standstill_wheel_speed_abs_mean_threshold")) &&
        (mode_.ax_abs_mean < readDoubleParam("standstill_ax_abs_mean_threshold")) &&
        (mode_.ay_abs_mean < readDoubleParam("standstill_ay_abs_mean_threshold")) &&
        (mode_.yaw_rate_abs_mean < readDoubleParam("standstill_yaw_rate_abs_mean_threshold")) &&
        (mode_.steer_abs_mean < readDoubleParam("standstill_abs_steer_mean_threshold")) &&
        (mode_.torque_cmd_abs_mean < readDoubleParam("standstill_torque_cmd_abs_mean_threshold"));

    flags_.straight_line =
        (!flags_.standstill) &&
        (state_.vx > readDoubleParam("straight_line_vx_min_threshold")) &&
        (straight_ay_abs_window_.mean() < readDoubleParam("straight_line_ay_abs_mean_threshold")) &&
        (straight_yaw_rate_abs_window_.mean() < readDoubleParam("straight_line_yaw_rate_abs_mean_threshold")) &&
        (straight_steer_abs_window_.mean() < readDoubleParam("straight_line_abs_steer_mean_threshold")) &&
        (straight_steer_dot_abs_window_.mean() < readDoubleParam("straight_line_abs_steer_dot_mean_threshold"));
}

void complementary_filter::applySoftStandstillMode(double dt)
{
    if (!readBoolParam("enable_soft_standstill_mode")) {
        return;
    }

    if (!flags_.standstill) {
        return;
    }

    state_.vx *= expDecayFactor(dt, readDoubleParam("standstill_vx_decay_tau_s"));
    state_.vy *= expDecayFactor(dt, readDoubleParam("standstill_vy_decay_tau_s"));
    state_.yaw_rate *= expDecayFactor(dt, readDoubleParam("standstill_r_decay_tau_s"));

    if (std::abs(raw_.wheel_speed_vx) < readDoubleParam("standstill_hard_zero_vx_threshold") &&
        mode_.torque_cmd_abs_mean < readDoubleParam("standstill_hard_zero_torque_cmd_threshold")) {
        state_.vx = 0.0;
        state_.vy = 0.0;
        state_.yaw_rate = 0.0;
        return;
    }

   

}

void complementary_filter::applySoftStraightLineMode(double dt)
{
    if (!readBoolParam("enable_soft_straight_line_mode")) {
        return;
    }

    if (!flags_.straight_line) {
        return;
    }

    state_.vy *= expDecayFactor(dt, readDoubleParam("straight_line_vy_decay_tau_s"));
    state_.yaw_rate *= expDecayFactor(dt, readDoubleParam("straight_line_r_decay_tau_s"));

    if (std::abs(state_.vy) < readDoubleParam("straight_line_hard_zero_vy_threshold") && 
        std::abs(state_.yaw_rate) < readDoubleParam("straight_line_hard_zero_r_threshold")) {
        state_.vy = 0.0;
        state_.yaw_rate = 0.0;
    }
    
}

void complementary_filter::integratePlanarPose(double dt)
{
    if (!readBoolParam("enable_debug_pose_integration")) {
        return;
    }

    state_.yaw = wrapAngle(state_.yaw + state_.yaw_rate * dt);

    const double cos_yaw = std::cos(state_.yaw);
    const double sin_yaw = std::sin(state_.yaw);

    const double vx_world = state_.vx * cos_yaw - state_.vy * sin_yaw;
    const double vy_world = state_.vx * sin_yaw + state_.vy * cos_yaw;

    state_.x += vx_world * dt;
    state_.y += vy_world * dt;
}

double complementary_filter::computeKinematicYawRate(double vx, double steer) const
{
    if (std::abs(vx) < readDoubleParam("min_vx_for_kinematic_model")) {
        return 0.0;
    }

    return vx / readDoubleParam("wheelbase") * std::tan(steer);
}

double complementary_filter::computeKinematicVyFromRearAxleZeroSlip(double r_ref) const
{
    return readDoubleParam("lr") * r_ref;
}

double complementary_filter::computeTorqueBasedLongitudinalAccel(double torque_cmd, double) const
{
    return readDoubleParam("ax_gain") * torque_cmd + readDoubleParam("ax_bias");
}

double complementary_filter::computeGravityBodyX() const
{
    const double c = std::cos(state_.yaw);
    const double s = std::sin(state_.yaw);

    return c * state_.g_x_init + s * state_.g_y_init;
}

double complementary_filter::computeGravityBodyY() const
{
    const double c = std::cos(state_.yaw);
    const double s = std::sin(state_.yaw);

    return -s * state_.g_x_init + c * state_.g_y_init;
}

double complementary_filter::filterSteerFirstOrder(double steer_raw, double dt)
{
    if (dt <= 0.0) {
        return steer_raw;
    }

    const double cutoff_hz = readDoubleParam("steer_dot_cutoff_hz");
    const double tau = 1.0 / (2.0 * M_PI * std::max(cutoff_hz, 1e-6));
    const double alpha = std::exp(-dt / tau);

    if (!steer_filter_state_.initialized) {
        steer_filter_state_.y = steer_raw;
        steer_filter_state_.initialized = true;
        return steer_raw;
    }

    steer_filter_state_.y = alpha * steer_filter_state_.y + (1.0 - alpha) * steer_raw;
    return steer_filter_state_.y;
}

double complementary_filter::computeSteerDotFromFilteredSteer(double steer_filtered, double dt)
{
    if (dt <= 0.0) {
        return 0.0;
    }

    if (!has_last_filtered_steer_) {
        last_filtered_steer_ = steer_filtered;
        has_last_filtered_steer_ = true;
        return 0.0;
    }

    const double steer_dot = (steer_filtered - last_filtered_steer_) / dt;
    last_filtered_steer_ = steer_filtered;
    return steer_dot;
}

double complementary_filter::readDoubleParam(const std::string& key) const
{
    return param_.get(key);
}

int complementary_filter::readIntParam(const std::string& key) const
{
    return static_cast<int>(std::lround(param_.get(key)));
}

bool complementary_filter::readBoolParam(const std::string& key) const
{
    return param_.get(key) > 0.5;
}

double complementary_filter::clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(x, hi));
}

double complementary_filter::clamp01(double x)
{
    return clamp(x, 0.0, 1.0);
}

double complementary_filter::lerp(double a, double b, double t)
{
    return a + (b - a) * clamp01(t);
}

double complementary_filter::softHardConfidence(double value, double soft_threshold, double hard_threshold)
{
    if (hard_threshold <= soft_threshold) {
        return (value <= soft_threshold) ? 1.0 : 0.0;
    }

    if (value <= soft_threshold) {
        return 1.0;
    }

    if (value >= hard_threshold) {
        return 0.0;
    }

    return (hard_threshold - value) / (hard_threshold - soft_threshold);
}

double complementary_filter::expDecayFactor(double dt, double tau)
{
    if (tau <= 1e-9) {
        return 0.0;
    }
    return std::exp(-dt / tau);
}

double complementary_filter::wrapAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

} // namespace complementary_filter_kinematic