#pragma once
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>

namespace kalman_filter_kinematic {

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

  double get(const std::string& name) const { return values.at(i(name)); }
  void set(const std::string& name, double v) { values.at(i(name)) = v; }
  size_t size() const { return values.size(); }
};

//// ============================================================
////  Pomocnicze funkcje do pobierania z JSON-a
//// ============================================================

// --- Wymagane pole double ---
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

// --- Wymagane pole bool ---
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

// --- Bezpieczna wersja z diagnostyką dla double ---
inline double JgetReqSafe(const nlohmann::json& J, const std::string& path) {
  try {
    return JgetReq(J, path);
  } catch (const std::exception& e) {
    std::cerr << "\n[JSON ERROR] while reading path: " << path
              << "\n  what(): " << e.what() << "\n" << std::endl;
    throw;
  }
}

// --- Bezpieczna wersja z diagnostyką dla bool ---
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
////  Budowa banku parametrów z pliku JSON
//// ============================================================
inline ParamBank build_param_bank(const nlohmann::json& J) {
  ParamBank P;

  //// ------------------------------------------------------------
  ////  Geometria / pojazd
  //// ------------------------------------------------------------
  const double lf = JgetReqSafe(J, "kalman_filter_kinematic.vehicle.lf");
  const double lr = JgetReqSafe(J, "kalman_filter_kinematic.vehicle.lr");

  P.add("kalman_filter_kinematic.vehicle.lf", lf);
  P.add("kalman_filter_kinematic.vehicle.lr", lr);
  P.add("kalman_filter_kinematic.vehicle.wheelbase", lf + lr);

  P.add("kalman_filter_kinematic.vehicle.mass",
        JgetReqSafe(J, "kalman_filter_kinematic.vehicle.mass"));

  P.add("kalman_filter_kinematic.vehicle.wheel_radius",
        JgetReqSafe(J, "kalman_filter_kinematic.vehicle.wheel_radius"));

  //// ------------------------------------------------------------
  ////  Logika / feature flagi
  //// ------------------------------------------------------------
  P.add("kalman_filter_kinematic.logic.enable_gravity_compensation",
        JgetReqBoolSafe(J, "kalman_filter_kinematic.logic.enable_gravity_compensation") ? 1.0 : 0.0);

  P.add("kalman_filter_kinematic.logic.enable_debug_pose_integration",
        JgetReqBoolSafe(J, "kalman_filter_kinematic.logic.enable_debug_pose_integration") ? 1.0 : 0.0);

  //// ------------------------------------------------------------
  ////  Proces / szum procesu
  //// ------------------------------------------------------------
  P.add("kalman_filter_kinematic.process_noise.vx",
        JgetReqSafe(J, "kalman_filter_kinematic.process_noise.vx"));

  P.add("kalman_filter_kinematic.process_noise.vy",
        JgetReqSafe(J, "kalman_filter_kinematic.process_noise.vy"));

  P.add("kalman_filter_kinematic.process_noise.r",
        JgetReqSafe(J, "kalman_filter_kinematic.process_noise.r"));

  P.add("kalman_filter_kinematic.process_noise.ax",
        JgetReqSafe(J, "kalman_filter_kinematic.process_noise.ax"));

  P.add("kalman_filter_kinematic.process_noise.ay",
        JgetReqSafe(J, "kalman_filter_kinematic.process_noise.ay"));

  //// ------------------------------------------------------------
  ////  P0 / kowariancja początkowa
  //// ------------------------------------------------------------
  P.add("kalman_filter_kinematic.initial_covariance.vx",
        JgetReqSafe(J, "kalman_filter_kinematic.initial_covariance.vx"));

  P.add("kalman_filter_kinematic.initial_covariance.vy",
        JgetReqSafe(J, "kalman_filter_kinematic.initial_covariance.vy"));

  P.add("kalman_filter_kinematic.initial_covariance.r",
        JgetReqSafe(J, "kalman_filter_kinematic.initial_covariance.r"));

  P.add("kalman_filter_kinematic.initial_covariance.ax",
        JgetReqSafe(J, "kalman_filter_kinematic.initial_covariance.ax"));

  P.add("kalman_filter_kinematic.initial_covariance.ay",
        JgetReqSafe(J, "kalman_filter_kinematic.initial_covariance.ay"));

  //// ------------------------------------------------------------
  ////  Prawdziwe pomiary
  //// ------------------------------------------------------------
  P.add("kalman_filter_kinematic.measurement_noise.imu_ax",
        JgetReqSafe(J, "kalman_filter_kinematic.measurement_noise.imu_ax"));

  P.add("kalman_filter_kinematic.measurement_noise.imu_ay",
        JgetReqSafe(J, "kalman_filter_kinematic.measurement_noise.imu_ay"));

  P.add("kalman_filter_kinematic.measurement_noise.imu_yaw_rate",
        JgetReqSafe(J, "kalman_filter_kinematic.measurement_noise.imu_yaw_rate"));

  P.add("kalman_filter_kinematic.measurement_noise.wheel_speed_vx",
        JgetReqSafe(J, "kalman_filter_kinematic.measurement_noise.wheel_speed_vx"));

  //// ------------------------------------------------------------
  ////  Pseudo-pomiary
  //// ------------------------------------------------------------
  P.add("kalman_filter_kinematic.pseudo_measurement_noise.kinematic_ay",
        JgetReqSafe(J, "kalman_filter_kinematic.pseudo_measurement_noise.kinematic_ay"));

  P.add("kalman_filter_kinematic.pseudo_measurement_noise.torque_ax",
        JgetReqSafe(J, "kalman_filter_kinematic.pseudo_measurement_noise.torque_ax"));
  P.add("kalman_filter_kinematic.pseudo_measurement_noise.zero_vy_standstill",
        JgetReqSafe(J, "kalman_filter_kinematic.pseudo_measurement_noise.zero_vy_standstill"));

  //// ------------------------------------------------------------
  ////  Model torque -> ax
  //// ------------------------------------------------------------
  P.add("kalman_filter_kinematic.torque_model.ax_gain",
        JgetReqSafe(J, "kalman_filter_kinematic.torque_model.ax_gain"));

  P.add("kalman_filter_kinematic.torque_model.ax_bias",
        JgetReqSafe(J, "kalman_filter_kinematic.torque_model.ax_bias"));

  //// ------------------------------------------------------------
  ////  Gravity / kalibracja i kompensacja
  //// ------------------------------------------------------------
  P.add("kalman_filter_kinematic.gravity.max_init_wheel_speed",
        JgetReqSafe(J, "kalman_filter_kinematic.gravity.max_init_wheel_speed"));

  P.add("kalman_filter_kinematic.gravity.max_init_yaw_rate",
        JgetReqSafe(J, "kalman_filter_kinematic.gravity.max_init_yaw_rate"));

  P.add("kalman_filter_kinematic.gravity.min_init_time",
        JgetReqSafe(J, "kalman_filter_kinematic.gravity.min_init_time"));

  //// ------------------------------------------------------------
  ////  Filtracja steer_dot
  //// ------------------------------------------------------------
  P.add("kalman_filter_kinematic.steer_dot_filter.cutoff_hz",
        JgetReqSafe(J, "kalman_filter_kinematic.steer_dot_filter.cutoff_hz"));

  P.add("kalman_filter_kinematic.steer_dot_filter.order",
        JgetReqSafe(J, "kalman_filter_kinematic.steer_dot_filter.order"));

  //// ------------------------------------------------------------
  ////  Dodatkowe zabezpieczenia / pomocnicze
  //// ------------------------------------------------------------
  P.add("kalman_filter_kinematic.numeric.min_vx_for_kinematic_model",
        JgetReqSafe(J, "kalman_filter_kinematic.numeric.min_vx_for_kinematic_model"));

  P.add("kalman_filter_kinematic.numeric.max_abs_steer_for_linear_region",
        JgetReqSafe(J, "kalman_filter_kinematic.numeric.max_abs_steer_for_linear_region"));


    P.add("kalman_filter_kinematic.numeric.standstill_vx_threshold",
        JgetReqSafe(J, "kalman_filter_kinematic.numeric.standstill_vx_threshold"));

  P.add("kalman_filter_kinematic.numeric.standstill_abs_steer_threshold",
        JgetReqSafe(J, "kalman_filter_kinematic.numeric.standstill_abs_steer_threshold"));

  P.add("kalman_filter_kinematic.numeric.standstill_abs_r_threshold",
        JgetReqSafe(J, "kalman_filter_kinematic.numeric.standstill_abs_r_threshold"));

  P.add("kalman_filter_kinematic.numeric.standstill_abs_torque_threshold",
        JgetReqSafe(J, "kalman_filter_kinematic.numeric.standstill_abs_torque_threshold"));

  return P;
}

} // namespace kalman_filter_kinematic