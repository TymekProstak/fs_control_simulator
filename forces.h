#ifndef FORCES_H
#define FORCES_H

#include "tire_force_model.h"

namespace metzler_model {


    struct forces_params {
        double drag_coefficient;
        double lift_coefficient;
        double wheelbase; // Distance between front and rear axles
        double wheel_distance;

    };

    
    struct traction_forces {
        double fx;      // Force in the x direction
        double fy;      // Force in the y direction
        double torque;  // Torque around the center of gravity
    }; 
    
    class Forces {
    public:
        Forces(const forces_params& params)
            : params_(params) {}

        traction_forces calculate_forces(double vx = 0.0, double vy = 0.0, double ax = 0.0, double ay = 0.0, ) ;

    private:
        forces_params params_;
        traction_forces forces_;

};
