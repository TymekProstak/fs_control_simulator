#ifndef VEHICLE_BODY_H
#define VEHICLE_BODY_H

#include "libs.h"
#include "forces.h"

namespace metzler_model {

    struct vehicle_body_state {
        double x;
        double y;
        double yaw;
        double vx;
        double vy;
        double yaw_rate;
        double ax;
        double ay;
        
    };

    struct vehicle_body_params {
        double mass; // masa pojazdu
        double inertia; // moment bezwładności pojazdu
        double wheelbase; // rozstaw osi
        double wheel_distance; // odległość między kołami
        double wheel_distance_front; // odległość między kołami z przodu
        double wheel_distance_rear; // odległość między kołami z tyłu
        double h_cg; // wysokość środka ciężkości
        double lf; // odległość środka ciężkości od przedniej osi
        double lr; // odległość środka ciężkości od tylnej osi

    };

    class VehicleBody {
        public:

            // Constructor 
            VehicleBody(const vehicle_body_params& params, const vehicle_body_state& initial_state = vehicle_body_state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
              

            /////////////////**** Setters ******* /////////////////////////////////////
            
            inline void set_state(const vehicle_body_state& new_state) {
                state_ = new_state;
            }
            inline void set_params(const vehicle_body_params& new_params) {
                params_ = new_params;
            }
            inline void set_x(double new_x) {
                state_.x = new_x;
            }
            
            inline void set_y(double new_y) {
                state_.y = new_y;
            }
            inline void set_yaw(double new_yaw) {
                state_.yaw = new_yaw;
            }

            inline void set_vx(double new_vx) {
                state_.vx = new_vx;
            }

            inline void set_vy(double new_vy) {
                state_.vy = new_vy;
            }

            inline void set_yaw_rate(double new_yaw_rate) {
                state_.yaw_rate = new_yaw_rate;
            }

            inline void set_ax(double new_ax) {
                state_.ax = new_ax;
            }

            inline void set_ay(double new_ay) {
                state_.ay = new_ay;
            }
            ///////////////////////////////////////////////////////////////////////


            //////////  ****** Getters ******* /////////////////////////////////////

            inline vehicle_body_state get_state() const {
                return state_;
            }

            inline vehicle_body_params get_params() const {
                return params_;
            }

            inline double get_x() const {
                return state_.x;
            }

            inline double get_y() const {
                return state_.y;
            }

            inline double get_yaw() const {
                return state_.yaw;
            }
            inline double get_vx() const {
                return state_.vx;
            }

            inline double get_vy() const {
                return state_.vy;
            }

            inline double get_yaw_rate() const {
                return state_.yaw_rate;
            }

            inline double get_ax() const {
                return state_.ax;
            }

            inline double get_ay() const {
                return state_.ay;
            }

            //////////////////////////////////////////////////////////////////////////////////////
            
            ///////  **** Dynamics  **** /////////////////

            void update_state(const forces_sumed& forces, double dt);


            //////////////////////////////////////////////////

            inline void reset(){

                state_ = vehicle_body_state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            }

            //// ********** vec to struct transforms ********* ////////////////////////////

            std::vector<double> state_to_vector() const ;

            void vector_to_state(const std::vector<double>& vec) ;
            

        

        

    private:
        vehicle_body_params params_;
        vehicle_body_state state_;
    };


} // namespace metzler_model


#endif // VEHICLE_COG_H