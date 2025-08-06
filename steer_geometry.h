#ifndef STEER_GEOMETRY_H
#define STEER_GEOMETRY_H

#include "libs.h"
#include "vehicle_body.h"
#include "engine.h"

namespace metzler_model {


        /// **** //// strukutura z parametrami geometrii układu kierowniczego //////

    struct steer_geometry_params {

        double wheelbase; // rozstaw osi
        double wheel_distance_front; // odległość między kołami z przodu
        double wheel_distance_rear; // odległość między kołami z tyłu
        double wheel_distance_avg; // średnia odległość między kołami
        double max_steering_angle; // maksymalny kąt skrętu

        double lf; // odległość środka ciężkości od przedniej osi
        double lr; // odległość środka ciężkości od tylnej osi
        
        double T; // czas reakcji układu kierowniczego -> eg kolumny kierowniczej
        double max_steering_rate; // maksymalna prędkość zmiany kąta skrętu
        double min_steering_rate; // minimalna prędkość zmiany kąta skrętu

    };
    ////****//////////////////////////////////////////////////////////////////




    struct steer_geometry_state {

        double front_left_steer_angle;
        double front_right_steer_angle;
        double steer_actual; // aktualny kąt skrętu na kolumnie kierowniczej


        double front_left_slip_angle; // kąt poślizgu przedniego lewego koła
        double front_right_slip_angle; // kąt poślizgu przedniego prawego koła
        double rear_left_slip_angle; // kąt poślizgu tylnego lewego koła
        double rear_right_slip_angle; // kąt poślizgu tylnego prawego koła
    };



    class SteerGeometry {
        
    public:
        SteerGeometry(const steer_geometry_params& params, const steer_geometry_state& initial_state = steer_geometry_state{0.0, 0.0})
            : params_(params), state_(initial_state) {}





        //////***** Gettery /////////////////////////

        inline steer_geometry_state get_state() const { return state_; }


        inline double get_front_left_steer_angle() const {
            return state_.front_left_steer_angle;
        }

        inline double get_front_right_steer_angle() const {
            return state_.front_right_steer_angle;
        }

        inline double get_steer_actual() const {
            return state_.steer_actual;
        }

        inline double get_front_left_slip_angle() const {
            return state_.front_left_slip_angle;
        }     
        inline double get_front_right_slip_angle() const {
            return state_.front_right_slip_angle;
        }

        inline double get_rear_left_slip_angle() const {
            return state_.rear_left_slip_angle;
        }

        inline double get_rear_right_slip_angle() const {
            return state_.rear_right_slip_angle;
        }

        //////////////////////////////////////////////



        //////***** Settery /////////////////////////


        inline void set_state(const steer_geometry_state& new_state) {
            state_ = new_state;
        }

        inline void set_front_left_steer_angle(double angle) {
            state_.front_left_steer_angle = angle;
        }

        inline void set_front_right_steer_angle(double angle) {
            state_.front_right_steer_angle = angle;
        }

        inline void set_steer_actual(double angle) {
            state_.steer_actual = angle;
        }


        inline void set_front_left_slip_angle(double angle) {
            state_.front_left_slip_angle = angle;
        }

        inline void set_front_right_slip_angle(double angle) {
            state_.front_right_slip_angle = angle;
        }

        inline void set_rear_left_slip_angle(double angle) {
            state_.rear_left_slip_angle = angle;
        }

        inline void set_rear_right_slip_angle(double angle) {
            state_.rear_right_slip_angle = angle;
        }

        //////////////////////////////////////////////


        ////***** Transformacje wektorów i struktur ********////
        
        inline vector<double> state_to_vector() const {
            return {state_.front_left_steer_angle, state_.front_right_steer_angle, state_.steer_actual,
                    state_.front_left_slip_angle, state_.front_right_slip_angle, state_.rear_left_slip_angle, state_.rear_right_slip_angle};
        }
        inline void vector_to_state(const vector<double>& vec) {
            if (vec.size() != 7) {
                throw std::invalid_argument("Vector size must be 7");
            }
            state_.front_left_steer_angle = vec[0];
            state_.front_right_steer_angle = vec[1];
            state_.steer_actual = vec[2];
            state_.front_left_slip_angle = vec[3];
            state_.front_right_slip_angle = vec[4];
            state_.rear_left_slip_angle = vec[5];
            state_.rear_right_slip_angle = vec[6];
        }

        //////////////////////////////////////////////



         void calculate_slip_angles( double vx, double vy , double yaw_rate) {;
         void calculate_steer_angles ();









        /// do cpp 

        void calculate_slip_angles( double vx = 0.0, double vy = 0.0, double yaw_rate = 0.0) {

            // Calculate the slip angles for each wheel based on the vehicle's velocity and yaw rate
           


            state_.rear_left_slip_angle = std::atan2(vy - params_.lr_cg * yaw_rate, vx - yaw_rate * params_.wheel_distance_rear / 2.0);
            state_.rear_right_slip_angle = std::atan2(vy - params_.lr_cg * yaw_rate, vx + yaw_rate * params_.wheel_distance_rear / 2.0);


            state_.front_left_slip_angle = std::atan2(vy + params_.lf_cg * yaw_rate, vx - yaw_rate * params_.wheel_distance_front / 2.0) - state_.front_left_steer_angle;
            state_.front_right_slip_angle = std::atan2(vy + params_.lf_cg * yaw_rate, vx + yaw_rate * params_.wheel_distance_front / 2.0) - state_.front_right_steer_angle; ;

            
            //ensure slip angles are within the range of -pi to pi

            state_.rear_left_slip_angle = std::fmod(state_.rear_left_slip_angle + M_PI, 2 * M_PI) - M_PI;
            state_.rear_right_slip_angle = std::fmod(state_.rear_right_slip_angle + M_PI, 2 * M_PI) - M_PI;
            state_.front_left_slip_angle = std::fmod(state_.front_left_slip_angle + M_PI, 2 * M_PI) - M_PI;
            state_.front_right_slip_angle = std::fmod(state_.front_right_slip_angle + M_PI, 2 * M_PI) - M_PI;


        }

        void calculate_steer_angles( ) {
             
            // for now assuming an ideal ackerman steering geometry
            


        }

        double derivative_steer(double steer_input) const {
            
            // 1 rzędowy układ kierowniczy
            double derivative = (steer_input - state_.steer_actual) / params_.T;

            // TODO : Jakieś filtry górno/dolno przepustowe ? 

            return derivative;
        }
        
        inline void update(double steer_input, double dt) {
            // Calculate the derivative of the steer angle
            double derivative = derivative_steer(steer_input);
            
            // Update the state using Euler's method
            state_.steer_actual += derivative * dt;


    private:
        steer_geometry_params params_;
        steer_geometry_state state_;
        
    


     };




} // namespace metzler_model


# endif // STEER_GEOMETRY_H