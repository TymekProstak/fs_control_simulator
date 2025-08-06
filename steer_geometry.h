#ifndef STEER_GEOMETRY_H
#define STEER_GEOMETRY_H

#include "libs.h"
#include "vehicle_cog.h"
#include "engine.h"

namespace metzler_model {

    struct steer_geometry_params {
        double wheelbase; // rozstaw osi
        double wheel_distance; // odległość między kołami
        double max_steering_angle; // maksymalny kąt skrętu
        double lf_cg; // odległość środka ciężkości od przedniej osi
        double lr_cg; // odległość środka ciężkości od tylnej osi
        double T; // czas reakcji układu kierowniczego
        double max_steering_rate; // maksymalna prędkość zmiany kąta skrętu
        double min_steering_rate; // minimalna prędkość zmiany kąta skrętu

    };

    struct steer_geometry_state {
        double front_left_steer_angle;
        double front_right_steer_angle;
    };
    class SteerGeometry {
    public:
        SteerGeometry(const steer_geometry_params& params, const steer_geometry_state& initial_state = steer_geometry_state{0.0, 0.0})
            : params_(params), state_(initial_state) {}

        steer_geometry_state get_state() const { return state_; }

        inline void set_state(const steer_geometry_state& new_state) {
            state_ = new_state;
        }

        inline double get_front_left_steer_angle() const {
            return state_.front_left_steer_angle;
        }

        inline double get_front_right_steer_angle() const {
            return state_.front_right_steer_angle;
        }


     };
}// namespace metzler_model


# endif // STEER_GEOMETRY_H