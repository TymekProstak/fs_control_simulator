#include "tire.h"

namespace metzler_model {


    Tire::Tire(const tire_params& params, const tire_state& initial_state)
        : params_(params), state_(initial_state) {}




    double Tire::calculate_velocity_along_tire(double vx, double vy, double yaw_rate) const {

        /** 
         * @brief Calculates the velocity along the tire based on kinematics.
         * @brief using the formula v_at_tire = v_cog + yaw_rate * r_tire_cog in a vector form
         * @param vx [m/s] Longitudinal velocity of the vehicle.
         * @param vy [m/s] Lateral velocity of the vehicle.
         * @param yaw_rate [rad/s] Yaw rate of the vehicle.
         * @return Velocity [m/s] along the tire.
        */ 
         
        // r is a vector from the center of gravity to the tire contact point -> will be used to calculate projection 
        double r_y = params_.y_from_cg;
        double r_x = params_.x_from_cg;

        // Calculate the velocity components

        double v_vehicle_y = vy + yaw_rate * r_x; // y component
        double v_vehicle_x = vx - yaw_rate * r_y; // x component


        double velocity_vehicle_in_contact_point = std::sqrt(v_vehicle_x * v_vehicle_x + v_vehicle_y * v_vehicle_y);

        // velocity with respet to longtidual axis of the tire

        double angle_with_longitudinal_axis = atan2(v_vehicle_y, v_vehicle_x);

        double angle_with_tire_longitudinal_axis = angle_with_longitudinal_axis - state_.steer_angle;

        // Normalize the angle to the range [-pi, pi]
        angle_with_tire_longitudinal_axis = std::fmod(angle_with_tire_longitudinal_axis + M_PI, 2 * M_PI) - M_PI;

        // Calculate the velocity along the tire
        double velocity_along_tire = velocity_vehicle_in_contact_point * std::cos(angle_with_tire_longitudinal_axis);

        return velocity_along_tire;

    }

    double Tire::calculate_slip_ratio(double vx, double vy, double yaw_rate) const {

        double velocity_along_tire = calculate_velocity_along_tire(vx, vy, yaw_rate);

        double slip_ratio  =  (state_.omega * params_.radius_effective - velocity_along_tire)/( std::abs(velocity_along_tire) + 1e-6); // adding a small value to avoid division by zero

        return slip_ratio;

    }

    double Tire::calculate_slip_angle(double vx, double vy, double yaw_rate) const {

        /** 
         * @brief Calculates the velocity along the tire based on kinematics.
         * @brief using the formula v_at_tire = v_cog + yaw_rate * r_tire_cog in a vector form
         * @param vx [m/s] Longitudinal velocity of the vehicle.
         * @param vy [m/s] Lateral velocity of the vehicle.
         * @param yaw_rate [rad/s] Yaw rate of the vehicle.
         * @return Velocity [m/s] along the tire.
        */ 
         
        // r is a vector from the center of gravity to the tire contact point -> will be used to calculate projection 
        double r_y = params_.y_from_cg;
        double r_x = params_.x_from_cg;

        // Calculate the velocity components

        double v_vehicle_y = vy + yaw_rate * r_x; // y component
        double v_vehicle_x = vx - yaw_rate * r_y; // x component

        // velocity with respect to longitudinal axis of the tire

        double angle_with_longitudinal_axis = atan2(v_vehicle_y, v_vehicle_x);

        // slip angle is the angle between the tire's longitudinal axis and the velocity vector
        double slip_angle = angle_with_longitudinal_axis - state_.steer_angle;

        // Normalize the slip angle to the range [-pi, pi]
        slip_angle = std::fmod(slip_angle + M_PI, 2 * M_PI) - M_PI;

        return slip_angle;



    }

    void Tire::calculate_and_set_slip_angle(double vx, double vy, double yaw_rate) {
        /**
         * @brief Calculates and sets the slip angle of the tire in the state.  
         * @param vx [m/s] Longitudinal velocity of the vehicle.
         * @param vy [m/s] Lateral velocity of the vehicle.
         * @param yaw_rate [rad/s] Yaw rate of the vehicle.
         */
        double slip_angle = calculate_slip_angle(vx, vy, yaw_rate);
        set_slip_angle(slip_angle);
    }

    void Tire::calculate_and_set_slip_ratio(double vx, double vy, double yaw_rate) {
        /**
         * @brief Calculates and sets the slip ratio of the tire in the state.
         * @param vx [m/s] Longitudinal velocity of the vehicle.
         * @param vy [m/s] Lateral velocity of the vehicle.
         * @param yaw_rate [rad/s] Yaw rate of the vehicle.
         */
        double slip_ratio = calculate_slip_ratio(vx, vy, yaw_rate);
        set_slip_ratio(slip_ratio);
    }

    double Tire::calculate_omega_derivative(double engine_torque, double tractive_force) const {
        /**
         * @brief Calculates the derivative of the tire's angular velocity.
         * @param engine_torque [Nm] Torque applied by the engine.
         * @param tractive_force [N] Tractive force at the tire-road interface.
         * @return Angular acceleration [rad/s^2].
         */
        double inertia_moment = params_.inertia_moment_effective;
        return (engine_torque - tractive_force * params_.radius_effective) / inertia_moment;
    }

    void Tire::update_omega(double engine_torque, double tractive_force, double dt){

        double derivative = calculate_omega_derivative(engine_torque, tractive_force);
        state_.omega += derivative * dt;
    }

    std::vector<double> Tire::state_to_vector() const {
        /**
         * @brief Converts the tire state to a vector representation.
         * @return Vector containing the tire state parameters.
         */
        return {state_.omega, state_.slip_angle, state_.slip_ratio, state_.steer_angle};
    }

    void Tire::vector_to_state(const std::vector<double>& state_vector) {
        /**
         * @brief Converts a vector representation to the tire state.
         * @param state_vector Vector containing the tire state parameters.
         */
        if (state_vector.size() != 4) {
            throw std::invalid_argument("State vector must have exactly 4 elements.");
        }
        state_.omega = state_vector[0];
        state_.slip_angle = state_vector[1];
        state_.slip_ratio = state_vector[2];
        state_.steer_angle = state_vector[3];
    }


} // namespace metzler_model





    















}