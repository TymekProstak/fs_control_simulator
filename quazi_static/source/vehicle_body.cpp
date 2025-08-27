#include "vehicle_body.h"

namespace metzler_model{

    VehicleBody::VehicleBody(const vehicle_body_params& params, const vehicle_body_state& initial_state)
                : params_(params), state_(initial_state) {}



    void VehicleBody::update_state( const forces_sumed& forces, double dt){

        // Update the state based on the forces and time step
           

            // NIE WOLNO ZMIEŃAĆ KOLEJNOŚCI OPERACJI, bo to jest układ równań różniczkowych


            state_.x += state_.vx * dt;
            state_.y += state_.vy * dt;

            state_.yaw += state_.yaw_rate * dt;


            state_.ax = (forces.fx ) / params_.mass;  // longitudinal acceleration in INERTIAL
            state_.ay = (forces.fy ) / params_.mass;  // lateral acceleration in INERTIAL

            double vx_dot =  state_.ax + state_.vy * state_.yaw_rate ;  // longitudinal acceleration in VEHICLE frame
            double vy_dot = state_.ay - state_.vx * state_.yaw_rate;  // lateral acceleration in VEHICLE frame

            state_.vx += vx_dot * dt;  // Update longitudinal velocity in VEHICLE frame
            state_.vy += vy_dot * dt;  // Update lateral velocity in VEHICLE frame
            
            // Update yaw rate based on the torque and inertia

            state_.yaw_rate = (forces.torque / params_.inertia);

    }


    std::vector<double> VehicleBody::state_to_vector() const {

        std::vector<double> result ;

        result.push_back(state_.x);
        result.push_back(state_.y);
        result.push_back(state_.yaw);
        result.push_back(state_.vx);
        result.push_back(state_.vy);
        result.push_back(state_.yaw_rate);
        result.push_back(state_.ax);
        result.push_back(state_.ay);

        return result;

    }


    void VehicleBody::vector_to_state(const std::vector<double>& vec) {

                if (vec.size() == 8) {
                    state_ = vehicle_body_state{
                        vec[0], vec[1], vec[2], vec[3], vec[4], vec[5], vec[6], vec[7]
                    };
                }
    }


} // namespace metzler_model