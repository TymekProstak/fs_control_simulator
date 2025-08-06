#ifndef NORMAL_MODEL_FORCES_H
#define NORMAL_MODEL_FORCES_H

#include "libs.h"

namespace metzler_model {
    
    struct normal_model_forces_param {
        double downforce_coefficient; // Coefficient of downforce
        double mass; // Mass of the vehicle
        double wheelbase; // Wheelbase of the vehicle
        double wheel_distance; // Distance between the wheels
        double h_cg; // Height of the center of gravity
        double lf_cg; // Distance from the center of gravity to the front axle
        double lr_cg; // Distance from the center of gravity to the rear axle
        double gravity = 9.81; // Gravitational acceleration
        double hf; // Height of the front roll center
        double hr; // Height of the rear roll center
        double front_roll_stiffness; // Front roll stiffness -> Kf
        double rear_roll_stiffness; // Rear roll stiffness  -> Kr
        double h_prim; // distance from the center of gravity to the front roll center
        
    };
    struct normal_model_forces {
        double front_left_force;  // Force on the front left wheel
        double front_right_force;  // Force on the front right wheel
        double rear_left_force;   // Force on the rear left wheel
        double rear_right_force;  // Force on the rear right wheel
    };


    normal_model_forces calculate_normal_model_forces(const normal_model_forces_para& params_, double vx= 0.0, double vy = 0.0 , double ax = 0.0, double ay = 0.0 );


    // przeniesc do cpp



    class NormalModelForces {
    public:
        NormalModelForces(const normal_model_forces_para& params_)
            : params_(param) {}




        normal_model_forces calculate_normal_forces(double vx = 0.0, double vy = 0.0, double ax = 0.0, double ay = 0.0) const ;

        ///// getters /////

        inline normal_model_forces_param get_param() const {
            return param_;
        }

        inline normal_model_forces get_forces() const {
            return forces_;
        }

        inline double  get_left_front_force() const {
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

        //////////////////////////////////////


        ///// setters /////


        inline void set_para(const normal_model_forces_params& new_param) {
            para_ = new_param;
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
        
        ////////////////////////////


        /// vec - struct transforms///
         inline vector<double> forces_to_vector() const {
            return {forces_.front_left_force, forces_.front_right_force, forces_.rear_left_force, forces_.rear_right_force};
        }

        inline void vector_to_forces(const vector<double>& forces_vector) {
            if (forces_vector.size() != 4) {
                throw std::invalid_argument("forces_vector must have exactly 4 elements.");
            }
            forces_.front_left_force = forces_vector[0];
            forces_.front_right_force = forces_vector[1];
            forces_.rear_left_force = forces_vector[2];
            forces_.rear_right_force = forces_vector[3];
        }
        /////////////////////////////////

    private:
        normal_model_forces_para para_;
        normal_model_forces forces_;
    };

    normal_model_forces calculate_normal_model_forces( double vx= 0.0,  double vy = 0.0 , double ax = 0.0, double ay = 0.0 ) {
        
        normal_model_forces forces;

        normal_model_forces static_forces;
        normal_model_forces  mass_transfer_forces;
        normal_model_forces aerodynamic_forces;
        normal_model_forces quazi_static_roll_forces;


        //////********** Calculate static forces ******* ////////////////////////////////////////////////////////////////////////////////////


        static_forces.front_left_force = params_.mass * para.gravity * (param.lr_cg / (para.lf_cg + para.lr_cg));
        static_forces.front_right_force = params.mass * params_.gravity * (para.lr_cg / (params_.lf_cg + params_.lr_cg));
        static_forces.rear_left_force = para.mass * param.gravity * (params.lf_cg / (param.lf_cg + param.lr_cg));
        static_forces.rear_right_force = params_.mass * para.gravity * (param.lf_cg / (para.lf_cg + para.lr_cg));


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////



        
        ////////****** Calculate mass transfer forces ****** //////////////////////////////////////////////////////////////////////////////////
        
        // Assuming ax is the longitudinal acceleration and ay is the lateral acceleration


        ///**** logitudal effects****///// 

        double mass_transfer = param.mass * params.gravity * (params_.h_cg / para.wheelbase) * (ax / param.gravity);
        
        mass_transfer_forces.front_left_force = -mass_transfer * (params_.lf_cg / (params_.lf_cg + params_.lr_cg));
        mass_transfer_forces.front_right_force = -mass_transfer * (params_.lf_cg / (params_.lf_cg + params_.lr_cg));
        mass_transfer_forces.rear_left_force = mass_transfer * (params_.lr_cg / (params_.lf_cg + params_.lr_cg));
        mass_transfer_forces.rear_right_force = mass_transfer * (params_.lr_cg / (params_.lf_cg + params_.lr_cg));


        ///**** lateral effects ****/////

        double lateral_mass_transfer = param.mass * params.gravity * (params_.h_cg / params_.wheel_distance) * (ay / param.gravity);

        mass_transfer_forces.front_left_force += lateral_mass_transfer * (params_.lf_cg / (params_.lf_cg + params_.lr_cg));
        mass_transfer_forces.front_right_force += lateral_mass_transfer * (params_.lf_cg / (params_.lf_cg + params_.lr_cg));
        mass_transfer_forces.rear_left_force -= lateral_mass_transfer * (params_.lr_cg / (params_.lf_cg + params_.lr_cg));
        mass_transfer_forces.rear_right_force -= lateral_mass_transfer * (params_.lr_cg / (params_.lf_cg + params_.lr_cg));


        /////////////////////////////////////////////////////////////////////////////////////////////////





        ///////******* Calculate aerodynamic forces **********//////////////////////////////////////////////////////////////////

        // TODO w przyszłości : rozszrzenie modelu o efekty z cfd/ przechyły itd

        double aerodynamic_force = params_.downforce_coefficient * params_.mass * (vx * vx );

        aerodynamic_forces.front_left_force = 1/2.0 * aerodynamic_force  * params_.lf_cg / params_.wheelbase;
        aerodynamic_forces.front_right_force = 1/2.0 * aerodynamic_force  * params_.lf_cg / params_.wheelbase;
        aerodynamic_forces.rear_left_force = 1/2.0 * aerodynamic_force * params_.lr_cg / params_.wheelbase;
        aerodynamic_forces.rear_right_force = 1/2.0 * aerodynamic_force  * params_.lr_cg / params_.wheelbase;


        //////////////////////////////////////////////////////////////////////////////////////////////////



        ///////******* Calculate quasi-static roll forces **********//////////////////////////////////////////////////////////////////


        











        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




        forces.front_left_force =  static_forces.front_left_force + mass_transfer_forces.front_left_force + aerodynamic_forces.front_left_force + quazi_static_roll_forces.front_left_force;
        forces.front_right_force =  static_forces.front_right_force + mass_transfer_forces.front_right_force + aerodynamic_forces.front_right_force + quazi_static_roll_forces.front_right_force;
        forces.rear_left_force =  static_forces.rear_left_force + mass_transfer_forces.rear_left_force + aerodynamic_forces.rear_left_force + quazi_static_roll_forces.rear_left_force;
        forces.rear_right_force =  static_forces.rear_right_force + mass_transfer_forces.rear_right_force + aerodynamic_forces.rear_right_force + quazi_static_roll_forces.rear_right_force;
    


        return forces;
    }
}
#endif // NORMAL_MODEL_FORCES_H