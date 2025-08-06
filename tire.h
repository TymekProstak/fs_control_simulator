#ifndef TIRE_H
#define TIRE_H

#include "libs.h"
namespace metzler_model {
    
    struct tire_state {
        double slip_angle;
        double slip_ratio;
        double omega;  // postive is anti- clockwise
        double delta_steer; // Steering angle with respect to the vehicle longitudinal axis
        double normal_force; // Normal force on the tire
    };

    struct tire_params{

        double inertia_moment_effective;
        double radius_effective;

    }

    // postive force is forward

    inline double tire_omega_derivative(const tire_state& state, const tire_params& params, double  traction_force, double dt) {
        
        double moment = traction_force * params.radius_effective;

        double derivative =  moment/ params.inertia_moment_effective;

        return derivative;

        
    }


    class Tire {
    public:
        Tire(const tire_params& params, const tire_state& initial_state = tire_state{0.0, 0.0, 0.0, 0.0 , 0.0})
            : params_(params), state_(initial_state) {}
            
        tire_state get_state() const { return state_; }
        inline void set_state(const tire_state& new_state) {
            state_ = new_state;
        }

        inline double get_slip_angle() const {
            return state_.slip_angle;
        }

        inline double get_slip_ratio() const {
            return state_.slip_ratio;
        }

        inline double get_omega() const {
            return state_.omega;
        }

        inline void update(double traction_force, double dt) {
            // Calculate the derivative of the tire omega
            double derivative = tire_omega_derivative(state_, params_, traction_force, dt);
            // Update the state using Euler's method
            state_.omega += derivative * dt;
        }

        inline void reset(const tire_state& initial_state = tire_state{0.0, 0.0, 0.0, 0.0, 0.0}) {
            state_ = initial_state;
        }

        inline void set_params(const tire_params& new_params) {
            params_ = new_params;
        }

        inline void set_slip_angle(double slip_angle) {
            state_.slip_angle = slip_angle;
        }

        inline void set_slip_ratio(double slip_ratio) {
            state_.slip_ratio = slip_ratio;
        }

        inline void set_omega(double omega) {
            state_.omega = omega;
        }
        inline void set_delta_steer(double delta_steer) {
            state_.delta_steer = delta_steer;
        }
        inline void set_normal_force(double normal_force) {
            state_.normal_force = normal_force;
        }  

    private:
        tire_params params_;
        tire_state state_;
    };

}