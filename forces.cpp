#include "forces.h"
#include <cmath>

namespace metzler_model {

Forces::Forces(const forces_params& params)
    : params_(params), forces_{}, RR_tire_force{}, FL_tire_force{}, FR_tire_force{}, RL_tire_force{},
      areo_downforce(0.0), areo_drag(0.0), FL_tire_torque(0.0), FR_tire_torque(0.0), RL_tire_torque(0.0), RR_tire_torque(0.0) {}

double Forces::calculate_aero_downforce(double vx) const {
    // Simple quadratic model: Downforce = coeff * vx^2
    return params_.downforce_coefficient * vx * vx;
}

void Forces::calculate_and_set_aero_downforce(double vx) {
    areo_downforce = calculate_aero_downforce(vx);
}

double Forces::calculate_aero_drag(double vx) const {
    // Simple quadratic model: Drag = coeff * vx^2
    return params_.drag_coefficient * vx * vx;
}

void Forces::calculate_and_set_aero_drag(double vx) {
    areo_drag = calculate_aero_drag(vx);
}

double Forces::calculate_FL_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) const {
    // Torque = Fy * x - Fx * y (relative to CoG)
    return new_force.fy * params_.FL_x_position - new_force.fx * params_.FL_y_position;
}

double Forces::calculate_FR_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) const {
    return new_force.fy * params_.FR_x_position - new_force.fx * params_.FR_y_position;
}

double Forces::calculate_RL_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) const {
    return new_force.fy * params_.RL_x_position - new_force.fx * params_.RL_y_position;
}

double Forces::calculate_RR_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) const {
    return new_force.fy * params_.RR_x_position - new_force.fx * params_.RR_y_position;
}

void Forces::calculate_and_set_FL_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) {
    FL_tire_torque = calculate_FL_tire_torque(new_force);
}

void Forces::calculate_and_set_FR_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) {
    FR_tire_torque = calculate_FR_tire_torque(new_force);
}

void Forces::calculate_and_set_RL_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) {
    RL_tire_torque = calculate_RL_tire_torque(new_force);
}

void Forces::calculate_and_set_RR_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) {
    RR_tire_torque = calculate_RR_tire_torque(new_force);
}

double Forces::calculate_set_and_return_aero_downforce(double vx) {
    areo_downforce = calculate_aero_downforce(vx);
    return areo_downforce;
}

double Forces::calculate_set_and_return_aero_drag(double vx) {
    areo_drag = calculate_aero_drag(vx);
    return areo_drag;
}

double Forces::calculate_set_and_return_FL_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) {
    FL_tire_torque = calculate_FL_tire_torque(new_force);
    return FL_tire_torque;
}

double Forces::calculate_set_and_return_FR_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) {
    FR_tire_torque = calculate_FR_tire_torque(new_force);
    return FR_tire_torque;
}

double Forces::calculate_set_and_return_RL_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) {
    RL_tire_torque = calculate_RL_tire_torque(new_force);
    return RL_tire_torque;
}

double Forces::calculate_set_and_return_RR_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) {
    RR_tire_torque = calculate_RR_tire_torque(new_force);
    return RR_tire_torque;
}

forces_sumed Forces::calculate_forces_sumed(
    double vx,
    tire_force_model_output_in_vehicle_frame& FL_force,
    tire_force_model_output_in_vehicle_frame& FR_force,
    tire_force_model_output_in_vehicle_frame& RL_force,
    tire_force_model_output_in_vehicle_frame& RR_force
) const {
    forces_sumed result;
    // Use calculate_set_and_return_* to update and get values
    double aero_drag = const_cast<Forces*>(this)->calculate_set_and_return_aero_drag(vx);

    double FL_torque = const_cast<Forces*>(this)->calculate_set_and_return_FL_tire_torque(FL_force);
    double FR_torque = const_cast<Forces*>(this)->calculate_set_and_return_FR_tire_torque(FR_force);
    double RL_torque = const_cast<Forces*>(this)->calculate_set_and_return_RL_tire_torque(RL_force);
    double RR_torque = const_cast<Forces*>(this)->calculate_set_and_return_RR_tire_torque(RR_force);

    result.fx = FL_force.fx + FR_force.fx + RL_force.fx + RR_force.fx - aero_drag;
    result.fy = FL_force.fy + FR_force.fy + RL_force.fy + RR_force.fy;
    result.torque = FL_torque + FR_torque + RL_torque + RR_torque;
    return result;
}

void Forces::calculate_and_set_forces_sumed(
    double vx,
    tire_force_model_output_in_vehicle_frame& FL_force,
    tire_force_model_output_in_vehicle_frame& FR_force,
    tire_force_model_output_in_vehicle_frame& RL_force,
    tire_force_model_output_in_vehicle_frame& RR_force
) {
    forces_ = calculate_forces_sumed(vx, FL_force, FR_force, RL_force, RR_force);
}

forces_sumed Forces::calculate_set_and_return_forces_sumed(
    double vx,
    tire_force_model_output_in_vehicle_frame& FL_force,
    tire_force_model_output_in_vehicle_frame& FR_force,
    tire_force_model_output_in_vehicle_frame& RL_force,
    tire_force_model_output_in_vehicle_frame& RR_force
) {
    forces_ = calculate_forces_sumed(vx, FL_force, FR_force, RL_force, RR_force);
    return forces_;
}

} // namespace metzler_model