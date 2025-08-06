#ifndef NORMAL_MODEL_FORCES_H
#define NORMAL_MODEL_FORCES_H

#include "libs.h"

namespace metzler_model {
    
    struct normal_model_forces_params {
        double lift_coefficient; // Coefficient of lift
        double mass; // Mass of the vehicle
        double wheelbase; // Wheelbase of the vehicle
        double wheel_distance; // Distance between the wheels
        double h_cg; // Height of the center of gravity
        double lf_cg; // Distance from the center of gravity to the front axle
        double lr_cg; // Distance from the center of gravity to the rear axle
        double gravity = 9.81; // Gravitational acceleration
        double 
    };
    struct normal_model_forces {
        double front_left_force;  // Force on the front left wheel
        double front_right_force;
        double rear_left_force;   // Force on the rear left wheel
        double rear_right_force;
    };

    inline normal_model_forces calculate_normal_model_forces(const normal_model_forces_params& params, double speed = 0.0, double ax = 0.0, double ay = 0.0 ) {
        normal_model_forces forces;

        forces.front_left_force = 
        forces.front_right_force =
        forces.rear_left_force =
        forces.rear_right_force =  


        return forces;
    }
}
#endif // NORMAL_MODEL_FORCES_H