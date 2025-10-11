#pragma once
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>

namespace lem_dynamics_sim_{

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

// bezpieczne pobieranie z JSON (wymagane)
inline double JgetReq(const nlohmann::json& J, const std::string& path){
  auto pos = path.find('.');
  if (pos == std::string::npos) {
    if (!J.contains(path)) throw std::runtime_error("JSON: missing required key '"+path+"'");
    return J.at(path).get<double>();
  }
  std::string head = path.substr(0,pos);
  std::string tail = path.substr(pos+1);
  if (!J.contains(head)) throw std::runtime_error("JSON: missing object '"+head+"'");
  return JgetReq(J.at(head), tail);
}

// opcjonalne z domyślną
inline double JgetOpt(const nlohmann::json& J, const std::string& path, double def){
  try { return JgetReq(J, path); } catch(...) { return def; }
}

// budowa banku parametrów na podstawie JSON
inline ParamBank build_param_bank(const nlohmann::json& J){
  ParamBank P;

  // --- Vehicle dynamics ---
  P.add("g",   JgetReq(J,"vehicle.g"));
  P.add("m",   JgetReq(J,"vehicle.m"));
  P.add("h",   JgetReq(J,"vehicle.h"));
  P.add("w",   JgetReq(J,"vehicle.w"));
  P.add("b",   JgetReq(J,"vehicle.b"));
  P.add("h1_roll", JgetReq(J,"vehicle.h1_roll"));
  P.add("h2_roll", JgetReq(J,"vehicle.h2_roll"));
  
  double a_val = J.contains("vehicle") && J["vehicle"].contains("a")
                   ? J["vehicle"]["a"].get<double>()
                   : (P.get("w") - P.get("b"));
  P.add("a", a_val);
  P.add("t_front",   JgetReq(J,"vehicle.T_front"));
  P.add("t_rear",   JgetReq(J,"vehicle.T_rear"));
  P.add("K1",  JgetReq(J,"vehicle.K1"));
  P.add("K2",  JgetReq(J,"vehicle.K2"));
  P.add("Cd",  JgetReq(J,"vehicle.Cd"));
  P.add("Cl1", JgetReq(J,"vehicle.Cl1"));
  P.add("Cl2", JgetReq(J,"vehicle.Cl2"));
  P.add("Cr",  JgetReq(J,"vehicle.Cr"));
  P.add("Iz",  JgetReq(J,"vehicle.Iz"));
  P.add("R",   JgetReq(J,"vehicle.R"));
  P.add("I_tire",  JgetReq(J,"vehicle.I_tire"));
  P.add("angle_construction_front",  JgetReq(J,"vehicle.angle_construction_front"));
  P.add("angle_construction_rear",  JgetReq(J,"vehicle.angle_construction_rear"));
  P.add("r_front",JgetReq(J,"vehicle.r_front"));
  P.add("r_rear",JgetReq(J,"vehicle.r_rear"));

  // --- Tire ---
  P.add("pCx1",    JgetReq(J,"tire.pCx1"));
  P.add("pDx1",    JgetReq(J,"tire.pDx1"));
  P.add("pDx2",    JgetReq(J,"tire.pDx2"));
  P.add("pEx1",    JgetReq(J,"tire.pEx1"));
  P.add("pKx1",    JgetReq(J,"tire.pKx1"));
  P.add("pKx3",    JgetReq(J,"tire.pKx3"));
  P.add("lambda_x",JgetReq(J,"tire.lambda_x"));
  P.add("pCy1",    JgetReq(J,"tire.pCy1"));
  P.add("pDy1",    JgetReq(J,"tire.pDy1"));
  P.add("pDy2",    JgetReq(J,"tire.pDy2"));
  P.add("pEy1",    JgetReq(J,"tire.pEy1"));
  P.add("pKy1",    JgetReq(J,"tire.pKy1"));
  P.add("pKy2",    JgetReq(J,"tire.pKy2"));
  P.add("lambda_y",JgetReq(J,"tire.lambda_y"));
  P.add("N0",      JgetReq(J,"tire.N0"));
  P.add("epsilon",  JgetReq(J,"tire.epsilon"));
  P.add("relax_time_slip_angle_first_guees", JgetReq(J,"tire.relax_length_slip_angle_piorek"));
  P.add("relax_time_slip_ratio_first_guees") = 0.3 * P.get("relax_length_slip_angle_first_guees"); // przybliżenie z dupy z wykresu stąd : https://www.researchgate.net/publication/3415307_Tire_Dynamic_Deflection_and_Its_Impact_on_Vehicle_Longitudinal_Dynamics_and_Control
  P.add("cy",     JgetReq(J,"tire.cy_first_guees")); // wartosc z dupy stąd https://research.tue.nl/en/studentTheses/analysis-and-development-of-formula-student-racing-tyres
  P.add("dy") =   P.get("cy")* P.get("relax_time_slip_angle_first_guees");// z dupy z długośći relaksacji i z teorii systemów 2 rzędu 
  P.add("cx",      JgetReq(J,"tire.cx_first_guees")); // wartosc z dupy stąd https://research.tue.nl/en/studentTheses/analysis-and-development-of-formula-student-racing-tyres
  P.add("dx") =  P.get("cx") * P.get("relax_time_slip_ratio_first_guees");// z dupy z długośći relaksacji i z teorii systemów 2 rzędu 


 // --- Drivetrain dynamics ---
 P.add("P_max_drive" , JgetReq(J, " drivetrain.P_max_drive"));
 P.add("P_min_recup" , JgetReq(J, "drivetrain.P_min_recup"));
 P.add("drivetrain_timescale",JgetReq(J, "drivetrain.drivetrain_timescale"));

 // --- Steering system ( eg. maxon) and anit acekrman dynamics/kineamtics --- 
 P.add("steering_system_timescale",JgetReq(J, "steering_system.steering_system_timescale"));
 P.add("steering_system_damping",JgetReq(J, "steering_system.steering_system_damping"));
 P.add("delta_u", JgetReq(J, "steering_system.delta_u"));
 P.add("delta_d", JgetReq(J, "steering_system.delta_d"));
 P.add("delta_min_increement", JgetReq(J, "steering_system.delta_min_increement"));
 P.add("natural_frequency_steering_system" = 1/P.get("steering_system_timescale"));

  // --- Simulation and model transition params ---
  P.add("time_step",  JgetReq(J,"simulation.time_step "));
  P.add("only_kin_speed_up_boundary",  JgetReq(J,"simulation.only_kin_speed_up_boundary "));
  P.add("only_mixed_speed_low_boundary",  JgetReq(J,"simulation.only_mixed_speed_low_boundary "));
  P.add("only_mixed_speed_up_boundary",  JgetReq(J,"simulation.only_mixed_speed_up_boundary "));
  P.add("only_dyn_speed_low_boundary",  JgetReq(J,"simulation.only_mixed_speed_low_boundary "));
  
  // --- DV Board and PID for wheel speed control ---
  P.add("delay_from_control_due_dv_board_can", JgetReq(J, "dv_board.delay_from_control_due_dv_board_can"));
  P.add(" . ")
  P.add("wheel_encoder_reading_time_step", JgetReq(J, "dv_board.wheel_encoder_reading_time_steep"));
  P.add("pid_loop_time_step", JgetReq(J, "dv_board.pid_loop_time_stemp"));
  P.add("pid_p", JgetReq(J, "dv_board.pid_p"));
  P.add("pid_i", JgetReq(J, "dv_board.pid_i"));
  P.add("pid_d", JgetReq(J, "dv_board.pid_d"));

  // Pose from DRI
  P.add("pose_latency", JgetReq(J, "pose.pose_latency"));
  P.add("pose_noise", JgetReq(J, "pose.pose_noise"));
  P.add("vel_latency", JgetReq(J, "pose.vel_latency"));
  P.add("vel_noise", JgetReq(J, "pose.vel_noise"));
  P.add("orientation_latency", JgetReq(J, "pose.orientation_latency"));
  P.add("orientation_noise", JgetReq(J, "pose.orientation_noise"));
  P.add("rotation_latency", JgetReq(J, "pose.rotation_latency"));
  P.add("rotation_noise", JgetReq(J, "pose.rotation_noise"));

  // vision to slam pipe  - > latency of cones ??
  

  // reszta będzie szła po prostu normalnie i latencje niepewność itd. zadane slam -> path, path -> control

  // głowna niewiadomo i co będzie wpywac na performce to jak modelwoac niepewność do slam oraz DRI ( można zmierzyć latecnje będzie trudniej)


  return P;
}
} // namespace lem_dyn_sim