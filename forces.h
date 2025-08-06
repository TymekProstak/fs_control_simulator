#ifndef FORCES_H
#define FORCES_H

namespace metzler_model {


    struct forces_params {
        double drag_coefficient;
        double lift_coefficient;
        double wheelbase; // Distance between front and rear axles
        double wheel_distance;

    };

    
    struct all_forces {
        double fx;      // Force in the x direction
        double fy;      // Force in the y direction
        double torque;  // Torque around the center of gravity
    }; 
    
    inline all_forces calculate_forces(const forces_params& params, double speed = 0.0, double fron) {


}