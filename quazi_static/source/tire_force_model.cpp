#include "tire_force_model.h"
#include <cmath>

namespace metzler_model {

TireForceModel::TireForceModel(const tire_forces_model_params& params)
    : params_(params), input_{}, output_{}, output_in_vehicle_frame_{} {}

/**
 * @brief Simplified Pacejka formula: F = D * sin(C * atan(B * x))
 * For lateral force: x = slip_angle, for longitudinal: x = slip_ratio
 * For lateral, E parameter is subtracted: F_lat = D_lat * sin(C_lat * atan(B_lat * slip_angle)) - E_long
 */
tire_force_model_output TireForceModel::calculate_tire_force(double slip_ratio, double slip_angle, double normal_force, double steer_angle) const {
    tire_force_model_output result;

    // Longitudinal force (Pacejka Magic Formula, simplified)
    result.longitudinal_force = params_.D_long * normal_force *
        std::sin(params_.C_long * std::atan(params_.B_long * slip_ratio));

    // Lateral force (Pacejka Magic Formula, simplified, minus E_long)
    result.lateral_force = params_.D_lat * normal_force *
        std::sin(params_.C_lat * std::atan(params_.B_lat * slip_angle)) - params_.E_long;

    return result;
}

tire_force_model_output TireForceModel::calculate_tire_force(const tire_force_model_input& input) const {
    return calculate_tire_force(input.slip_ratio, input.slip_angle, input.normal_force, input.steer_angle);
}

tire_force_model_output_in_vehicle_frame TireForceModel::calculate_tire_force_in_vehicle_frame(double slip_ratio, double slip_angle, double normal_force, double steer_angle) const {
    tire_force_model_output forces = calculate_tire_force(slip_ratio, slip_angle, normal_force, steer_angle);

    tire_force_model_output_in_vehicle_frame result;
    // Rotate the result by steer_angle
    result.fx = forces.longitudinal_force * std::cos(steer_angle) - forces.lateral_force * std::sin(steer_angle);
    result.fy = forces.longitudinal_force * std::sin(steer_angle) + forces.lateral_force * std::cos(steer_angle);

    return result;

}

tire_force_model_output_in_vehicle_frame TireForceModel::calculate_tire_force_in_vehicle_frame(const tire_force_model_input& input) const {
    return calculate_tire_force_in_vehicle_frame(input.slip_ratio, input.slip_angle, input.normal_force, input.steer_angle);
}

void TireForceModel::calculate_and_set_tire_force(const tire_force_model_input& input) {
    output_ = calculate_tire_force(input);
    input_ = input;
}

void TireForceModel::calculate_and_set_tire_force(double slip_ratio, double slip_angle, double normal_force, double steer_angle) {
    output_ = calculate_tire_force(slip_ratio, slip_angle, normal_force, steer_angle);
    input_ = {slip_ratio, slip_angle, normal_force, steer_angle};
}

void TireForceModel::calculate_and_set_tire_force_in_vehicle_frame(double slip_ratio, double slip_angle, double normal_force, double steer_angle) {
    output_in_vehicle_frame_ = calculate_tire_force_in_vehicle_frame(slip_ratio, slip_angle, normal_force, steer_angle);
    input_ = {slip_ratio, slip_angle, normal_force, steer_angle};
}

void TireForceModel::calculate_and_set_tire_force_in_vehicle_frame(const tire_force_model_input& input) {
    output_in_vehicle_frame_ = calculate_tire_force_in_vehicle_frame(input);
    input_ = input;
}

tire_force_model_output TireForceModel::calculate_set_and_return_tire_force(const tire_force_model_input& input) {
    output_ = calculate_tire_force(input);
    input_ = input;
    return output_;
}

tire_force_model_output TireForceModel::calculate_set_and_return_tire_force(double slip_ratio, double slip_angle, double normal_force, double steer_angle) {
    output_ = calculate_tire_force(slip_ratio, slip_angle, normal_force, steer_angle);
    input_ = {slip_ratio, slip_angle, normal_force, steer_angle};
    return output_;
}

tire_force_model_output_in_vehicle_frame TireForceModel::calculate_set_and_return_tire_force_in_vehicle_frame(const tire_force_model_input& input) {
    output_in_vehicle_frame_ = calculate_tire_force_in_vehicle_frame(input);
    input_ = input;
    return output_in_vehicle_frame_;
}

tire_force_model_output_in_vehicle_frame TireForceModel::calculate_set_and_return_tire_force_in_vehicle_frame(double slip_ratio, double slip_angle, double normal_force, double steer_angle) {
    output_in_vehicle_frame_ = calculate_tire_force_in_vehicle_frame(slip_ratio, slip_angle, normal_force, steer_angle);
    input_ = {slip_ratio, slip_angle, normal_force, steer_angle};
    return output_in_vehicle_frame_;
}

} // namespace metzler_model