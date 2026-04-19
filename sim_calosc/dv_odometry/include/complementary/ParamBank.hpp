#pragma once

#include <nlohmann/json.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>

namespace complementary_filter_kinematic {

//// ============================================================
////  Struktura głównego banku parametrów
//// ============================================================
struct ParamBank {
    std::vector<std::string> names;
    std::unordered_map<std::string, int> idx;
    std::vector<double> values;

    int add(const std::string& name, double val) {
        auto it = idx.find(name);
        if (it != idx.end()) {
            values[it->second] = val;
            return it->second;
        }
        int k = static_cast<int>(names.size());
        names.push_back(name);
        idx[name] = k;
        values.push_back(val);
        return k;
    }

    int i(const std::string& name) const {
        auto it = idx.find(name);
        if (it == idx.end()) {
            throw std::runtime_error("ParamBank: missing key '" + name + "'");
        }
        return it->second;
    }

    double get(const std::string& name) const {
        return values.at(i(name));
    }

    void set(const std::string& name, double v) {
        values.at(i(name)) = v;
    }

    size_t size() const {
        return values.size();
    }
};

//// ============================================================
////  Pomocnicze funkcje do pobierania z JSON-a
//// ============================================================

// ------------------------------------------------------------
// Wymagane pole typu double
// ------------------------------------------------------------
inline double JgetReq(const nlohmann::json& J, const std::string& path) {
    auto pos = path.find('.');
    if (pos == std::string::npos) {
        if (!J.contains(path)) {
            throw std::runtime_error("JSON: missing required key '" + path + "'");
        }
        return J.at(path).get<double>();
    }

    const std::string head = path.substr(0, pos);
    const std::string tail = path.substr(pos + 1);

    if (!J.contains(head)) {
        throw std::runtime_error("JSON: missing object '" + head + "'");
    }

    return JgetReq(J.at(head), tail);
}

// ------------------------------------------------------------
// Wymagane pole typu bool
// ------------------------------------------------------------
inline bool JgetReqBool(const nlohmann::json& J, const std::string& path) {
    auto pos = path.find('.');
    if (pos == std::string::npos) {
        if (!J.contains(path)) {
            throw std::runtime_error("JSON: missing required key '" + path + "'");
        }
        return J.at(path).get<bool>();
    }

    const std::string head = path.substr(0, pos);
    const std::string tail = path.substr(pos + 1);

    if (!J.contains(head)) {
        throw std::runtime_error("JSON: missing object '" + head + "'");
    }

    return JgetReqBool(J.at(head), tail);
}

// ------------------------------------------------------------
// Wersja safe dla double
// ------------------------------------------------------------
inline double JgetReqSafe(const nlohmann::json& J, const std::string& path) {
    try {
        return JgetReq(J, path);
    } catch (const std::exception& e) {
        std::cerr << "\n[JSON ERROR] while reading path: " << path
                  << "\n  what(): " << e.what() << "\n" << std::endl;
        throw;
    }
}

// ------------------------------------------------------------
// Wersja safe dla bool
// ------------------------------------------------------------
inline bool JgetReqBoolSafe(const nlohmann::json& J, const std::string& path) {
    try {
        return JgetReqBool(J, path);
    } catch (const std::exception& e) {
        std::cerr << "\n[JSON ERROR] while reading bool path: " << path
                  << "\n  what(): " << e.what() << "\n" << std::endl;
        throw;
    }
}

//// ============================================================
////  Budowa banku parametrów z JSON-a
//// ============================================================
inline ParamBank build_param_bank(const nlohmann::json& J) {
    ParamBank P;

    const std::string base = "complementary_filter_kinematic";

    auto D = [&](const std::string& path) -> double {
        return JgetReqSafe(J, base + "." + path);
    };

    auto B = [&](const std::string& path) -> double {
        return JgetReqBoolSafe(J, base + "." + path) ? 1.0 : 0.0;
    };

    //// ========================================================
    ////  LOGIC
    //// ========================================================
    P.add("enable_gravity_compensation",    B("logic.enable_gravity_compensation"));
    P.add("enable_debug_pose_integration",  B("logic.enable_debug_pose_integration"));
    P.add("enable_ins_fusion",              B("logic.enable_ins_fusion"));
    P.add("enable_r_kinematic_correction",  B("logic.enable_r_kinematic_correction"));
    P.add("enable_soft_standstill_mode",    B("logic.enable_soft_standstill_mode"));
    P.add("enable_soft_straight_line_mode", B("logic.enable_soft_straight_line_mode"));

    //// ========================================================
    ////  PREPROCESSING : IMU AX
    //// ========================================================
    P.add("imu_ax_history_size_samples",           D("preprocessing.imu_ax.history_size_samples"));
    P.add("imu_ax_relative_deviation_threshold",   D("preprocessing.imu_ax.relative_deviation_threshold"));
    P.add("imu_ax_relative_deviation_ref_epsilon", D("preprocessing.imu_ax.relative_deviation_ref_epsilon"));
    P.add("imu_ax_absolute_reject_threshold",      D("preprocessing.imu_ax.absolute_reject_threshold"));
    P.add("imu_ax_max_consecutive_rejects",        D("preprocessing.imu_ax.max_consecutive_rejects"));

    //// ========================================================
    ////  PREPROCESSING : IMU AY
    //// ========================================================
    P.add("imu_ay_history_size_samples",           D("preprocessing.imu_ay.history_size_samples"));
    P.add("imu_ay_relative_deviation_threshold",   D("preprocessing.imu_ay.relative_deviation_threshold"));
    P.add("imu_ay_relative_deviation_ref_epsilon", D("preprocessing.imu_ay.relative_deviation_ref_epsilon"));
    P.add("imu_ay_absolute_reject_threshold",      D("preprocessing.imu_ay.absolute_reject_threshold"));
    P.add("imu_ay_max_consecutive_rejects",        D("preprocessing.imu_ay.max_consecutive_rejects"));

    //// ========================================================
    ////  PREPROCESSING : IMU YAW RATE
    //// ========================================================
    P.add("imu_yaw_rate_history_size_samples",           D("preprocessing.imu_yaw_rate.history_size_samples"));
    P.add("imu_yaw_rate_relative_deviation_threshold",   D("preprocessing.imu_yaw_rate.relative_deviation_threshold"));
    P.add("imu_yaw_rate_relative_deviation_ref_epsilon", D("preprocessing.imu_yaw_rate.relative_deviation_ref_epsilon"));
    P.add("imu_yaw_rate_absolute_reject_threshold",      D("preprocessing.imu_yaw_rate.absolute_reject_threshold"));
    P.add("imu_yaw_rate_max_consecutive_rejects",        D("preprocessing.imu_yaw_rate.max_consecutive_rejects"));

    //// ========================================================
    ////  HEURISTIC WINDOWS
    //// ========================================================
    P.add("ax_time",                         D("heuristic_windows.ax_time"));
    P.add("ay_time",                         D("heuristic_windows.ay_time"));
    P.add("torque_cmd_time",                 D("heuristic_windows.torque_cmd_time"));
    P.add("steer_dot_time",                  D("heuristic_windows.steer_dot_time"));
    P.add("r_kin_gyro_discrepancy_time",     D("heuristic_windows.r_kin_gyro_discrepancy_time"));

    //// ========================================================
    ////  MODE DETECTION WINDOWS
    //// ========================================================
    P.add("standstill_time",             D("mode_detection_windows.standstill_time"));
    P.add("straight_line_driving_time",  D("mode_detection_windows.straight_line_driving_time"));

    //// ========================================================
    ////  VEHICLE
    //// ========================================================
    P.add("lf",           D("vehicle.lf"));
    P.add("lr",           D("vehicle.lr"));
    P.add("mass",         D("vehicle.mass"));
    P.add("wheel_radius", D("vehicle.wheel_radius"));
    P.add("wheelbase",    D("vehicle.wheelbase"));

    //// ========================================================
    ////  SENSOR FREQUENCY
    //// ========================================================
    P.add("imu_freq",         D("sensor_freq.imu_freq"));
    P.add("wheel_speed_freq", D("sensor_freq.wheel_speed_freq"));
    P.add("torque_cmd_freq",  D("sensor_freq.torque_cmd_freq"));
    P.add("ins_freq",         D("sensor_freq.ins_freq"));
    P.add("steer_freq",       D("sensor_freq.steer_freq"));

    //// ========================================================
    ////  TORQUE MODEL
    //// ========================================================
    P.add("ax_gain", D("torque_model.ax_gain"));
    P.add("ax_bias", D("torque_model.ax_bias"));

    //// ========================================================
    ////  GRAVITY
    //// ========================================================
    P.add("max_init_wheel_speed", D("gravity.max_init_wheel_speed"));
    P.add("max_init_yaw_rate",    D("gravity.max_init_yaw_rate"));
    P.add("min_init_time",        D("gravity.min_init_time"));

    //// ========================================================
    ////  STEER DOT FILTER
    //// ========================================================
    P.add("steer_dot_cutoff_hz",    D("steer_dot_filter.cutoff_hz"));
    P.add("steer_dot_filter_order", D("steer_dot_filter.order"));

    //// ========================================================
    ////  R UPDATE
    //// ========================================================
    P.add("r_gyro_weight_min",                    D("r_update.gyro_weight_min"));
    P.add("r_gyro_weight_max",                    D("r_update.gyro_weight_max"));
    P.add("r_kinematic_weight_min",               D("r_update.kinematic_weight_min"));
    P.add("r_kinematic_weight_max",               D("r_update.kinematic_weight_max"));
    P.add("r_min_vx_for_kinematic_correction",    D("r_update.min_vx_for_kinematic_correction"));
    P.add("r_min_abs_steer_for_kinematic_correction", D("r_update.min_abs_steer_for_kinematic_correction"));
    P.add("r_steer_dot_soft_threshold",           D("r_update.steer_dot_soft_threshold"));
    P.add("r_steer_dot_hard_threshold",           D("r_update.steer_dot_hard_threshold"));
    P.add("r_ay_soft_threshold",                  D("r_update.ay_soft_threshold"));
    P.add("r_ay_hard_threshold",                  D("r_update.ay_hard_threshold"));
    P.add("r_discrepancy_soft_threshold",         D("r_update.discrepancy_soft_threshold"));
    P.add("r_discrepancy_hard_threshold",         D("r_update.discrepancy_hard_threshold"));

    //// ========================================================
    ////  VX UPDATE
    //// ========================================================
    P.add("vx_wheel_weight_min",            D("vx_update.wheel_weight_min"));
    P.add("vx_wheel_weight_max",            D("vx_update.wheel_weight_max"));
    P.add("vx_ax_activity_soft_threshold",  D("vx_update.ax_activity_soft_threshold"));
    P.add("vx_ax_activity_hard_threshold",  D("vx_update.ax_activity_hard_threshold"));
    P.add("vx_torque_cmd_soft_threshold",   D("vx_update.torque_cmd_soft_threshold"));
    P.add("vx_torque_cmd_hard_threshold",   D("vx_update.torque_cmd_hard_threshold"));
    P.add("vx_min_vx_for_wheel_update",     D("vx_update.min_vx_for_wheel_update"));

    //// ========================================================
    ////  VY UPDATE
    //// ========================================================
    P.add("vy_kinematic_weight_min",                      D("vy_update.kinematic_weight_min"));
    P.add("vy_kinematic_weight_max",                      D("vy_update.kinematic_weight_max"));
    P.add("vy_ay_activity_soft_threshold",                D("vy_update.ay_activity_soft_threshold"));
    P.add("vy_ay_activity_hard_threshold",                D("vy_update.ay_activity_hard_threshold"));
    P.add("vy_steer_dot_soft_threshold",                  D("vy_update.steer_dot_soft_threshold"));
    P.add("vy_steer_dot_hard_threshold",                  D("vy_update.steer_dot_hard_threshold"));
    P.add("vy_r_kin_gyro_discrepancy_soft_threshold",     D("vy_update.r_kin_gyro_discrepancy_soft_threshold"));
    P.add("vy_r_kin_gyro_discrepancy_hard_threshold",     D("vy_update.r_kin_gyro_discrepancy_hard_threshold"));
    P.add("vy_min_vx_for_kinematic_vy_reference",         D("vy_update.min_vx_for_kinematic_vy_reference"));

    //// ========================================================
    ////  STANDSTILL MODE
    //// ========================================================
    P.add("standstill_wheel_speed_abs_mean_threshold", D("standstill_mode.wheel_speed_abs_mean_threshold"));
    P.add("standstill_ax_abs_mean_threshold",          D("standstill_mode.ax_abs_mean_threshold"));
    P.add("standstill_ay_abs_mean_threshold",          D("standstill_mode.ay_abs_mean_threshold"));
    P.add("standstill_yaw_rate_abs_mean_threshold",    D("standstill_mode.yaw_rate_abs_mean_threshold"));
    P.add("standstill_abs_steer_mean_threshold",       D("standstill_mode.abs_steer_mean_threshold"));
    P.add("standstill_torque_cmd_abs_mean_threshold",  D("standstill_mode.torque_cmd_abs_mean_threshold"));
    P.add("standstill_vx_decay_tau_s",                 D("standstill_mode.vx_decay_tau_s"));
    P.add("standstill_vy_decay_tau_s",                 D("standstill_mode.vy_decay_tau_s"));
    P.add("standstill_r_decay_tau_s",                  D("standstill_mode.r_decay_tau_s"));
    P.add("standstill_hard_zero_vx_threshold",         D("standstill_mode.hard_zero_vx_threshold"));
    P.add("standstill_hard_zero_vy_threshold",         D("standstill_mode.hard_zero_vy_threshold"));
    P.add("standstill_hard_zero_r_threshold",          D("standstill_mode.hard_zero_r_threshold"));
    P.add("standstill_hard_zero_torque_cmd_threshold", D("standstill_mode.hard_zero_torque_cmd_threshold"));

    //// ========================================================
    ////  STRAIGHT LINE MODE
    //// ========================================================
    P.add("straight_line_vx_min_threshold",            D("straight_line_mode.vx_min_threshold"));
    P.add("straight_line_abs_steer_mean_threshold",    D("straight_line_mode.abs_steer_mean_threshold"));
    P.add("straight_line_abs_steer_dot_mean_threshold",D("straight_line_mode.abs_steer_dot_mean_threshold"));
    P.add("straight_line_ay_abs_mean_threshold",       D("straight_line_mode.ay_abs_mean_threshold"));
    P.add("straight_line_yaw_rate_abs_mean_threshold", D("straight_line_mode.yaw_rate_abs_mean_threshold"));
    P.add("straight_line_vy_decay_tau_s",              D("straight_line_mode.vy_decay_tau_s"));
    P.add("straight_line_r_decay_tau_s",               D("straight_line_mode.r_decay_tau_s"));
    P.add("straight_line_hard_zero_vy_threshold",      D("straight_line_mode.hard_zero_vy_threshold"));
    P.add("straight_line_hard_zero_r_threshold",       D("straight_line_mode.hard_zero_r_threshold"));

    //// ========================================================
    ////  NUMERIC
    //// ========================================================
    P.add("min_vx_for_kinematic_model", D("numeric.min_vx_for_kinematic_model"));
    P.add("max_abs_vy",                 D("numeric.max_abs_vy"));
    P.add("max_abs_r",                  D("numeric.max_abs_r"));
    P.add("clamp_negative_vx_to_zero",  B("numeric.clamp_negative_vx_to_zero"));

    return P;
}

} // namespace complementary_filter_kinematic