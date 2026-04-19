#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iostream>
#include <string>
#include <utility>

#include "complementary/ParamBank.hpp"

namespace complementary_filter_kinematic
{

struct RawInputs
{
    double imu_ax_raw = 0.0;         // [m/s^2]
    double imu_ay_raw = 0.0;         // [m/s^2]
    double imu_yaw_rate_raw = 0.0;   // [rad/s]

    double wheel_speed_vx = 0.0;     // [m/s]

    double steer_raw = 0.0;          // [rad]
    double steer_dot_raw = 0.0;      // [rad/s] obecnie nieużywane

    double torque_cmd = 0.0;         // [Nm]

    double ins_x = 0.0;              // [m]
    double ins_y = 0.0;              // [m]
    double ins_yaw = 0.0;            // [rad]
};

struct ProcessedSignals
{
    double imu_ax_preprocessed = 0.0;        // [m/s^2]
    double imu_ay_preprocessed = 0.0;        // [m/s^2]
    double imu_yaw_rate_preprocessed = 0.0;  // [rad/s]

    double imu_ax_comp = 0.0;                // [m/s^2]
    double imu_ay_comp = 0.0;                // [m/s^2]

    double steer_filtered = 0.0;             // [rad]
    double steer_dot_filtered = 0.0;         // [rad/s]

    double r_kin = 0.0;                      // [rad/s]
    double vy_kin = 0.0;                     // [m/s]
    double torque_ax_model = 0.0;            // [m/s^2]

    double r_kin_gyro_discrepancy = 0.0;     // [rad/s]
};

struct State
{
    double x = 0.0;                          // [m]
    double y = 0.0;                          // [m]
    double yaw = 0.0;                        // [rad]

    double vx = 0.0;                         // [m/s]
    double vy = 0.0;                         // [m/s]
    double yaw_rate = 0.0;                   // [rad/s]

    double gyro_bias = 0.0;                  // [rad/s]
    double ax_bias = 0.0;                    // [m/s^2]
    double ay_bias = 0.0;                    // [m/s^2]

    double g_x_init = 0.0;                   // [m/s^2]
    double g_y_init = 0.0;                   // [m/s^2]
    double g_in_plane_modul_init = 0.0;      // [m/s^2]
};

struct HeuristicSignals
{
    double ax_abs_mean = 0.0;
    double ay_abs_mean = 0.0;
    double torque_cmd_abs_mean = 0.0;
    double steer_dot_abs_mean = 0.0;
    double r_kin_gyro_discrepancy_abs_mean = 0.0;
};

struct ModeSignals
{
    double wheel_speed_abs_mean = 0.0;
    double ax_abs_mean = 0.0;
    double ay_abs_mean = 0.0;
    double yaw_rate_abs_mean = 0.0;
    double steer_abs_mean = 0.0;
    double steer_dot_abs_mean = 0.0;
    double torque_cmd_abs_mean = 0.0;
};

struct ComplementaryGains
{
    double r_gyro_weight = 1.0;
    double r_kinematic_weight = 0.0;

    double vx_wheel_weight = 0.0;
    double vy_kinematic_weight = 0.0;
};

struct ModeFlags
{
    bool standstill = false;
    bool straight_line = false;
};

struct FirstOrderLowPassState
{
    double y = 0.0;
    bool initialized = false;

    void reset()
    {
        y = 0.0;
        initialized = false;
    }
};

class TimeAveragingWindow
{
public:
    explicit TimeAveragingWindow(double window_time_s = 0.0);

    void setWindowTime(double window_time_s);
    void reset();

    void push(double value, double dt);
    double mean() const;
    bool empty() const;
    double totalTime() const;

private:
    void trimToWindow();

private:
    double window_time_s_ = 0.0;
    double weighted_sum_ = 0.0;
    double total_time_ = 0.0;
    std::deque<std::pair<double, double>> buffer_; // (dt, value)
};

class OutlierRejector
{
public:
    OutlierRejector() = default;

    void configure(int history_size_samples,
                   double relative_deviation_threshold,
                   double relative_deviation_ref_epsilon,
                   double absolute_reject_threshold,
                   int max_consecutive_rejects);

    void reset();

    double filter(double sample);
    bool initialized() const;

private:
    double referenceMean() const;
    void pushAccepted(double sample);

private:
    int history_size_samples_ = 1;
    double relative_deviation_threshold_ = 1.0;
    double relative_deviation_ref_epsilon_ = 1e-3;
    double absolute_reject_threshold_ = 1e9;
    int max_consecutive_rejects_ = 0;

    bool initialized_ = false;
    int consecutive_rejects_ = 0;
    double last_accepted_ = 0.0;
    std::deque<double> history_;
};

class complementary_filter
{
public:
    explicit complementary_filter(const ParamBank& param_bank);

    void setImuMeasurement(double ax_raw, double ay_raw, double yaw_rate_raw);
    void setWheelSpeedMeasurement(double vx_meas);
    void setSteeringMeasurement(double steer, double steer_dot = 0.0);
    void setTorqueCommand(double torque_command);
    void setInsPose(double x, double y, double yaw);

    void setInitialGravityInPlane(double gx_init, double gy_init);
    void setInitialGyroBias(double gyro_bias);

    void updateFromImu(double dt);
    void updateFromWheelSpeed();

    const State& getState() const;
    const RawInputs& getRawInputs() const;
    const ProcessedSignals& getProcessedSignals() const;
    const HeuristicSignals& getHeuristicSignals() const;
    const ModeSignals& getModeSignals() const;
    const ComplementaryGains& getGains() const;
    const ModeFlags& getModeFlags() const;

private:
    void preprocessImuMeasurements();
    void updateGravityCompensatedImu();
    void updateFilteredSteering(double dt);

    void updateKinematicReferences();
    void updateHeuristicWindows(double dt);
    void updateModeWindows(double dt);

    void updateComplementaryGains();
    void updateYawRateEstimate();
    void propagateLongitudinalVelocity(double dt);
    void propagateLateralVelocity(double dt);
    void updateLongitudinalVelocityFromWheel();

    void evaluateModes();
    void applySoftStandstillMode(double dt);
    void applySoftStraightLineMode(double dt);

    void integratePlanarPose(double dt);

    double computeKinematicYawRate(double vx, double steer) const;
    double computeKinematicVyFromRearAxleZeroSlip(double r_ref) const;
    double computeTorqueBasedLongitudinalAccel(double torque_cmd, double vx) const;

    double computeGravityBodyX() const;
    double computeGravityBodyY() const;

    double filterSteerFirstOrder(double steer_raw, double dt);
    double computeSteerDotFromFilteredSteer(double steer_filtered, double dt);

    double readDoubleParam(const std::string& key) const;
    int readIntParam(const std::string& key) const;
    bool readBoolParam(const std::string& key) const;

    static double clamp(double x, double lo, double hi);
    static double clamp01(double x);
    static double lerp(double a, double b, double t);
    static double softHardConfidence(double value, double soft_threshold, double hard_threshold);
    static double expDecayFactor(double dt, double tau);
    static double wrapAngle(double angle);

private:
    ParamBank param_;

    RawInputs raw_{};
    ProcessedSignals proc_{};
    State state_{};
    HeuristicSignals heur_{};
    ModeSignals mode_{};
    ComplementaryGains gains_{};
    ModeFlags flags_{};

    OutlierRejector imu_ax_rejector_{};
    OutlierRejector imu_ay_rejector_{};
    OutlierRejector imu_yaw_rate_rejector_{};

    TimeAveragingWindow ax_abs_window_{};
    TimeAveragingWindow ay_abs_window_{};
    TimeAveragingWindow torque_cmd_abs_window_{};
    TimeAveragingWindow steer_dot_abs_window_{};
    TimeAveragingWindow r_discrepancy_abs_window_{};

    TimeAveragingWindow standstill_wheel_speed_abs_window_{};
    TimeAveragingWindow standstill_ax_abs_window_{};
    TimeAveragingWindow standstill_ay_abs_window_{};
    TimeAveragingWindow standstill_yaw_rate_abs_window_{};
    TimeAveragingWindow standstill_steer_abs_window_{};
    TimeAveragingWindow standstill_steer_dot_abs_window_{};
    TimeAveragingWindow standstill_torque_abs_window_{};

    TimeAveragingWindow straight_ay_abs_window_{};
    TimeAveragingWindow straight_yaw_rate_abs_window_{};
    TimeAveragingWindow straight_steer_abs_window_{};
    TimeAveragingWindow straight_steer_dot_abs_window_{};

    FirstOrderLowPassState steer_filter_state_{};
    double last_filtered_steer_ = 0.0;
    bool has_last_filtered_steer_ = false;

    bool has_imu_ = false;
    bool has_wheel_speed_ = false;
    bool has_steer_ = false;
    bool has_torque_command_ = false;
    bool has_ins_ = false;
    bool has_gravity_init_ = false;
};

} // namespace complementary_filter_kinematic