#ifndef VEHICLE_H
#define VEHICLE_H

#include "types.h"

namespace metzler_model {

    struct  vehicle_cog_state {
        
        double X;
        double Y;
        double vx;
        double vy;
        double phi;
        double ax;
        double ay;
    };

    struct vehicle_input{

        double throttle;
        double brake;
        double steering_angle;
    };

    struct vehicle_params{

        double wheelbase; // distance between front and rear axles
        double h_cog; // height of center of gravity
        double m; // mass of the vehicle
        double Iz; // moment of inertia of the vehicle with respect to the vertical axis
        double lf; // distance from the center of gravity to the front axle
        double lr; // distance from the center of gravity to the rear axle
        double wheel_distance; // distance between the left and right wheels

    };

    



}
#endif // VEHICLE_H