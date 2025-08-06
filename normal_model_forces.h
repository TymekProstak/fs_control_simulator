#ifndef NORMAL_MODEL_FORCES_H
#define NORMAL_MODEL_FORCES_H

#include "libs.h"
#include <vector>
#include <stdexcept>

namespace metzler_model {

    ///***************///// PARAMS STRUKTURA ///////*****************///
    struct normal_model_forces_params {
        double drag_coefficient;
        double downforce_coefficient;
        double pressure_center_height;
        double wheelbase;
        double wheel_distance_front;
        double wheel_distance_rear;
        double wheel_distance_avg;
        double mass;
        double h_cg;
        double lf;
        double lr;
        double gravity = 9.81;
        double h_roll_f;
        double h_roll_r;
        double front_roll_stiffness;
        double rear_roll_stiffness;
    };

    ///***************///// FORCES STRUKTURA ///////*****************///
    struct normal_model_forces {
        double front_left_force;
        double front_right_force;
        double rear_left_force;
        double rear_right_force;
    };

    ///***************///// KLASA NORMALMODEL ///////*****************///
    class NormalModelForces {
    public:
        NormalModelForces(const normal_model_forces_params& params);

        normal_model_forces calculate_normal_forces(double vx = 0.0, double vy = 0.0, double ax = 0.0, double ay = 0.0) const;

        inline normal_model_forces_params get_params() const {
            return params_;
        }

        inline normal_model_forces get_forces() const {
            return forces_;
        }

        inline double get_left_front_force() const {
            return forces_.front_left_force;
        }
        inline double get_right_front_force() const {
            return forces_.front_right_force;
        }
        inline double get_left_rear_force() const {
            return forces_.rear_left_force;
        }
        inline double get_right_rear_force() const {
            return forces_.rear_right_force;
        }

        inline void set_params(const normal_model_forces_params& new_params) {
            params_ = new_params;
        }

        inline void set_forces(const normal_model_forces& new_forces) {
            forces_ = new_forces;
        }

        inline void set_left_front_force(double force) {
            forces_.front_left_force = force;
        }
        inline void set_right_front_force(double force) {
            forces_.front_right_force = force;
        }
        inline void set_left_rear_force(double force) {
            forces_.rear_left_force = force;
        }
        inline void set_right_rear_force(double force) {
            forces_.rear_right_force = force;
        }

        inline std::vector<double> forces_to_vector() const {
            return {forces_.front_left_force, forces_.front_right_force, forces_.rear_left_force, forces_.rear_right_force};
        }

        inline void vector_to_forces(const std::vector<double>& forces_vector) {
            if (forces_vector.size() != 4) {
                throw std::invalid_argument("forces_vector must have exactly 4 elements.");
            }
            forces_.front_left_force = forces_vector[0];
            forces_.front_right_force = forces_vector[1];
            forces_.rear_left_force = forces_vector[2];
            forces_.rear_right_force = forces_vector[3];
        }

    private:
        normal_model_forces_params params_;
        normal_model_forces forces_;
    };

} // namespace metzler_model

#endif // NORMAL_MODEL_FORCES_H