#ifndef TIRE_H
#define TIRE_H

#include "libs.h"
namespace metzler_model {
    
    struct tire_state {

        double slip_angle;
        double slip_ratio; 
        double omega;  // postive is anti- clockwise -> eg. negative with respect to forwad motion
        double steer_angle; // angle of the tire with respect to the vehicle body, positive is anti-clockwise


    };

    struct tire_params{

        double inertia_moment_effective; // effective inertia moment of the tire and  gear system
        double radius_effective; // effective radius of the tire -> in more advanced models may change dynamicly 
        double y_from_cg; // y cordinate with respect to  CoG xy
        double x_from_cg; // x cordinate with repsect to CoG

    };

    class Tire{

        public:

            // Constructor to initialize tire with default parameters
            Tire(const tire_params& params, const tire_state& initial_state = tire_state{0.0, 0.0, 0.0, 0.0}) ;
                


            ///////////// *** Setters *** //////////////////////////////

            // Set the tire state
            inline void set_state(const tire_state& state) {
                state_ = state;
            }


            // Set the tire parameters
            inline void set_params(const tire_params& params) {
                params_ = params;
            }

            // Set the slip angle
            inline void set_slip_angle(double slip_angle) {
                state_.slip_angle = slip_angle;
            }

            // Set the slip ratio
            inline void set_slip_ratio(double slip_ratio) {
                state_.slip_ratio = slip_ratio;
            }

            // Set the angular velocity
            inline void set_omega(double omega) {       
                state_.omega = omega;
            }   

            // Set the steer angle
            inline void set_steer_angle(double steer_angle) {
                state_.steer_angle = steer_angle;
            }


            ////////////// *** Getters *** //////////////////////////////

            // Get the tire state
            inline tire_state get_state() const {
                return state_;
            }   
            // Get the tire parameters
            inline tire_params get_params() const {
                return params_;
            }   
            // Get the slip angle
            inline double get_slip_angle() const {
                return state_.slip_angle;
            }       
            // Get the slip ratio
            inline double get_slip_ratio() const {      
                return state_.slip_ratio;
            }           
            // Get the angular velocity
            inline double get_omega() const {               
                return state_.omega;
            }
            // Get the steer angle
            inline double get_steer_angle() const {
                return state_.steer_angle;
            }
            /////////////////////////////////////////


            /////***** Dynamics and kinematics functions *****/ //////////////////////////////

            /**
             * @brief Calculates the velocity along the tire based on kinematics.
             * @param vx [m/s] Longitudinal velocity of the vehicle.
             * @param vy [m/s] Lateral velocity of the vehicle.
             * @param yaw_rate [rad/s] Yaw rate of the vehicle.
             * @return Velocity [m/s] along the tire.
             */

            double calculate_velocity_along_tire(double vx = 0.0, double vy = 0.0, double yaw_rate = 0.0) const;

            /**
             * @brief Calculates the slip ratio of the tire.
             * @param vx [m/s] Longitudinal velocity of the vehicle.
             * @param vy [m/s] Lateral velocity of the vehicle.
             * @param yaw_rate [rad/s] Yaw rate of the vehicle.
             * @return Slip ratio (dimensionless).
             */

            double calculate_slip_ratio(double vx = 0.0, double vy = 0.0, double yaw_rate = 0.0) const;

            /**
             * @brief Calculates and sets the slip ratio of the tire in the state.
             * @param vx [m/s] Longitudinal velocity of the vehicle.
             * @param vy [m/s] Lateral velocity of the vehicle.
             * @param yaw_rate [rad/s] Yaw rate of the vehicle.
             */
            void calculate_and_set_slip_ratio(double vx = 0.0, double vy = 0.0, double yaw_rate = 0.0);

            /**
             * @brief Calculates the slip angle of the tire.
             * @param vx [m/s] Longitudinal velocity of the vehicle.
             * @param vy [m/s] Lateral velocity of the vehicle.
             * @param yaw_rate [rad/s] Yaw rate of the vehicle.
             * @return Slip angle [rad].
             */

            double calculate_slip_angle(double vx = 0.0, double vy = 0.0, double yaw_rate = 0.0) const;

            /**
             * @brief Calculates and sets the slip angle of the tire in the state.
             * @param vx [m/s] Longitudinal velocity of the vehicle.
             * @param vy [m/s] Lateral velocity of the vehicle.
             * @param yaw_rate [rad/s] Yaw rate of the vehicle.
             */

            void calculate_and_set_slip_angle(double vx = 0.0, double vy = 0.0, double yaw_rate = 0.0);

            /**
             * @brief Calculates the derivative of the tire's angular velocity.
             * @param engine_torque [Nm] Torque applied by the engine.
             * @param tractive_force [N] Tractive force at the tire-road interface.
             * @return Angular acceleration [rad/s^2].
             */

            double calculate_omega_derivative(double engine_torque = 0.0, double tractive_force = 0.0) const;

            /**
             * @brief Updates the tire's angular velocity using the provided tractive force and time step.
             * @param tractive_force [N] Tractive force at the tire-road interface.
             * @param dt [s] Time step.
             */

            void update_omega(double engine_torque = 0.0 ,double tractive_force = 0.0 , double dt);

            //////////////////////////////////////////////////////////////////////////////////////////////////

            inline void reset_tire_state() {
                state_ = tire_state{0.0, 0.0, 0.0, 0.0};
            }


            ////// *** struct - vec transforms /////////////////////////////

            std::vector<double> state_to_vector() const ;

            void vector_to_state(const std::vector<double>& vec) ;

            inline void reset_tire_state() {
                state_ = tire_state{0.0, 0.0, 0.0, 0.0};
            }

        private:

            tire_state state_;
            tire_params params_;



    };

        

} // namespace metzler_model


#endif // TIRE_H