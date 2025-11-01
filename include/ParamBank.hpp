#pragma once
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>

namespace lem_dynamics_sim_ {

//// ============================================================
////  Struktura głównego banku parametrów
//// ============================================================
struct ParamBank {
  std::vector<std::string> names;                 // nazwy w kolejności
  std::unordered_map<std::string,int> idx;        // nazwa -> index
  std::vector<double> values;                     // wartości liczbowe

  int add(const std::string& name, double val){
    auto it = idx.find(name);
    if (it != idx.end()){
      values[it->second] = val;
      return it->second;
    }
    int k = (int)names.size();
    names.push_back(name);
    idx[name] = k;
    values.push_back(val);
    return k;
  }

  int i(const std::string& name) const {
    auto it = idx.find(name);
    if (it == idx.end()) throw std::runtime_error("ParamBank: missing key '"+name+"'");
    return it->second;
  }

  double get(const std::string& name) const { return values.at(i(name)); }
  void   set(const std::string& name, double v){ values.at(i(name)) = v; }
  size_t size() const { return values.size(); }
};

//// ============================================================
////  Pomocnicze funkcje do pobierania z JSON-a
//// ============================================================

// --- Wymagane pole ---
inline double JgetReq(const nlohmann::json& J, const std::string& path){
  auto pos = path.find('.');
  if (pos == std::string::npos) {
    if (!J.contains(path))
      throw std::runtime_error("JSON: missing required key '"+path+"'");
    return J.at(path).get<double>();
  }
  std::string head = path.substr(0,pos);
  std::string tail = path.substr(pos+1);
  if (!J.contains(head))
    throw std::runtime_error("JSON: missing object '"+head+"'");
  return JgetReq(J.at(head), tail);
}

// --- Bezpieczna wersja z diagnostyką ---
inline double JgetReqSafe(const nlohmann::json& J, const std::string& path) {
  try {
    return JgetReq(J, path);
  } catch (const std::exception& e) {
    std::cerr << "\n[JSON ERROR] while reading path: " << path
              << "\n  what(): " << e.what() << "\n" << std::endl;
    throw;  // przekazujemy wyjątek dalej, by zatrzymać node
  }
}

// --- Opcjonalne pole z wartością domyślną ---
inline double JgetOpt(const nlohmann::json& J, const std::string& path, double def){
  try { return JgetReq(J, path); } catch(...) { return def; }
}

//// ============================================================
////  Budowa banku parametrów z pliku JSON
//// ============================================================
inline ParamBank build_param_bank(const nlohmann::json& J){
  ParamBank P;

  //// --- Vehicle dynamics ---
  P.add("g",   JgetReqSafe(J,"vehicle.g"));
  P.add("m",   JgetReqSafe(J,"vehicle.m"));
  P.add("h",   JgetReqSafe(J,"vehicle.h"));
  P.add("w",   JgetReqSafe(J,"vehicle.w"));
  P.add("b",   JgetReqSafe(J,"vehicle.b"));
  P.add("h1_roll", JgetReqSafe(J,"vehicle.h1_roll"));
  P.add("h2_roll", JgetReqSafe(J,"vehicle.h2_roll"));

  double a_val = J.contains("vehicle") && J["vehicle"].contains("a")
                   ? J["vehicle"]["a"].get<double>()
                   : (P.get("w") - P.get("b"));
  P.add("a", a_val);

  P.add("t_front",   JgetReqSafe(J,"vehicle.T_front"));
  P.add("t_rear",    JgetReqSafe(J,"vehicle.T_rear"));
  P.add("K1",        JgetReqSafe(J,"vehicle.K1"));
  P.add("K2",        JgetReqSafe(J,"vehicle.K2"));
  P.add("Cd",        JgetReqSafe(J,"vehicle.Cd"));
  P.add("Cl1",       JgetReqSafe(J,"vehicle.Cl1"));
  P.add("Cl2",       JgetReqSafe(J,"vehicle.Cl2"));
  P.add("Cr",        JgetReqSafe(J,"vehicle.Cr"));
  P.add("Iz",        JgetReqSafe(J,"vehicle.Iz"));
  P.add("R",         JgetReqSafe(J,"vehicle.R"));
  P.add("I_tire",    JgetReqSafe(J,"vehicle.I_tire"));
  P.add("angle_construction_front", JgetReqSafe(J,"vehicle.angle_construction_front"));
  P.add("angle_construction_rear",  JgetReqSafe(J,"vehicle.angle_construction_rear"));
  P.add("r_front",   JgetReqSafe(J,"vehicle.r_front"));
  P.add("r_rear",    JgetReqSafe(J,"vehicle.r_rear"));

  //// --- Tire parameters ---
  P.add("pCx1",    JgetReqSafe(J,"tire.pCx1"));
  P.add("pDx1",    JgetReqSafe(J,"tire.pDx1"));
  P.add("pDx2",    JgetReqSafe(J,"tire.pDx2"));
  P.add("pEx1",    JgetReqSafe(J,"tire.pEx1"));
  P.add("pKx1",    JgetReqSafe(J,"tire.pKx1"));
  P.add("pKx3",    JgetReqSafe(J,"tire.pKx3"));
  P.add("lambda_x",JgetReqSafe(J,"tire.lambda_x"));
  P.add("pCy1",    JgetReqSafe(J,"tire.pCy1"));
  P.add("pDy1",    JgetReqSafe(J,"tire.pDy1"));
  P.add("pDy2",    JgetReqSafe(J,"tire.pDy2"));
  P.add("pEy1",    JgetReqSafe(J,"tire.pEy1"));
  P.add("pKy1",    JgetReqSafe(J,"tire.pKy1"));
  P.add("pKy2",    JgetReqSafe(J,"tire.pKy2"));
  P.add("lambda_y",JgetReqSafe(J,"tire.lambda_y"));
  P.add("N0",      JgetReqSafe(J,"tire.N0"));
  P.add("epsilon", JgetReqSafe(J,"tire.epsilon")); // regularzycja slipów zgodnie z Tire and Vehicle Dynamics książka Hans B. Pacejka , 185 strona
  P.add("relax_time_slip_angle_first_guees", JgetReqSafe(J,"tire.relax_length_slip_angle_piorek")); // inżynierka piórek rozdział o bocznej relaksacji
  P.add("relax_time_slip_ratio_first_guees", 0.3 * P.get("relax_time_slip_angle_first_guees")); // / przybliżenie z dupy z wykresu stąd : https://www.researchgate.net/publication/3415307_Tire_Dynamic_Deflection_and_Its_Impact_on_Vehicle_Longitudinal_Dynamics_and_Control
  P.add("cy", JgetReqSafe(J,"tire.cy_first_guees")); // // wartosc z dupy stąd https://research.tue.nl/en/studentTheses/analysis-and-development-of-formula-student-racing-tyres
  P.add("dy", P.get("cy") * P.get("relax_time_slip_angle_first_guees")); // z dupy z długośći relaksacji i z teorii systemów 2 rzędu 
  P.add("cx", JgetReqSafe(J,"tire.cx_first_guees")); // // wartosc z dupy stąd https://research.tue.nl/en/studentTheses/analysis-and-development-of-formula-student-racing-tyres
  P.add("dx", P.get("cx") * P.get("relax_time_slip_ratio_first_guees")); // z dupy z długośći relaksacji i z teorii systemów 2 rzędu

  //// --- Drivetrain ---
  P.add("P_max_drive",        JgetReqSafe(J,"drivetrain.P_max_drive"));
  P.add("P_min_recup",        JgetReqSafe(J,"drivetrain.P_min_recup"));
  P.add("drivetrain_timescale", JgetReqSafe(J,"drivetrain.drivetrain_timescale")); // z dupy fest
  P.add("max_torque",       0.95* JgetReqSafe(J,"drivetrain.max_torque")); // przyjęto 95% sprawności przekładni( do zmierzenia)
  P.add("min_torque", 0.95* JgetReqSafe(J,"drivetrain.min_torque")); // przyjęto 95% sprawności przekładni( do zmierzenia)
  //// --- Steering system ---
  P.add("natural_frequency_steering_system", JgetReqSafe(J,"steering_system.natural_frequency_steering_system"));
  P.add("steering_system_damping",           JgetReqSafe(J,"steering_system.steering_system_damping"));
  P.add("max_steer" , JgetReqSafe(J, "steering_system.max_steer"));
  P.add("min_steer", JgetReq(J, "steering_system.min_steer"));


  //// --- Simulation ---
  P.add("simulation_time_step", JgetReqSafe(J,"simulation.time_step"));

  //// --- DV board and PID ---
  P.add("steer_input_delay", JgetReqSafe(J,"dv_board.dealy_from_control_to_maxon_command"));
  P.add("torque_input_delay", JgetReqSafe(J,"dv_board.dealy_from_control_to_torque_command"));
  P.add("wheel_encoder_reading_time_step",     JgetReqSafe(J,"dv_board.wheel_encoder_reading_time_steep"));
  P.add("pid_time_step",                       JgetReqSafe(J,"dv_board.pid_time_step"));
  P.add("pid_p",                               JgetReqSafe(J,"dv_board.pid_p"));
  P.add("pid_i",                               JgetReqSafe(J,"dv_board.pid_i"));
  P.add("pid_d",                               JgetReqSafe(J,"dv_board.pid_d"));

  //// --- Pose / INS ---
  P.add("pose_noise",        JgetReqSafe(J,"pose.pose_noise"));
  P.add("orientation_noise", JgetReqSafe(J,"pose.orientation_noise"));
  P.add("rotation_noise",    JgetReqSafe(J,"pose.rotation_noise"));
  P.add("ins_frequancy",     JgetReqSafe(J,"pose.ins_frequancy"));

  //// --- Vision ---
  P.add("fov_x",                       JgetReqSafe(J,"vision.fov_x"));
  P.add("fov_y",                       JgetReqSafe(J,"vision.fov_y"));
  P.add("z_camera_to_cog",             JgetReqSafe(J,"vision.z_camera_to_cog"));
  P.add("x_camera_to_cog",             JgetReqSafe(J,"vision.x_camera_to_cog"));
  P.add("y_camera_to_cog",             JgetReqSafe(J,"vision.y_camera_to_cog"));
  P.add("cone_height",                 JgetReqSafe(J,"vision.cone_height"));
  P.add("frames_per_second",           JgetReqSafe(J,"vision.frames_per_second"));
  P.add("mean_time_of_vision_execuction", JgetReqSafe(J,"vision.mean_time_of_vision_execuction"));
  P.add("var_of_vision_time_execution",   JgetReqSafe(J,"vision.var_of_vision_time_execution"));
  P.add("camera_range",               JgetReqSafe(J,"vision.camera_range"));

  return P;
}

} // namespace lem_dynamics_sim_
