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
        double min_steering_angle; // minimalny kąt skrętu 

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

        double steer_angle_on_column; // aktualny kąt skrętu na kolumnie kierowniczej


        double front_left_slip_angle; // kąt poślizgu przedniego lewego koła
        double front_right_slip_angle; // kąt poślizgu przedniego prawego koła
        double rear_left_slip_angle; // kąt poślizgu tylnego lewego koła
        double rear_right_slip_angle; // kąt poślizgu tylnego prawego koła
    };

    struct slip_angles {
        double front_left_slip_angle;
        double front_right_slip_angle;
        double rear_left_slip_angle;
        double rear_right_slip_angle;
    };

    struct steer_angles { 

        double front_left_steer_angle; // kąt skrętu przedniego lewego koła
        double front_right_steer_angle; // kąt skrętu przedniego prawego koła
        double steer_angle_on_column; // aktualny kąt skrętu na kolumnie kierowniczej
    };



    class SteerGeometry {
        
    public:


        ///***************///// KLASA STEERGEOMETRY DEFINICJE ///////*****************///

        /// Konstruktor klasy SteerGeometry
        /// @param params Struktura z parametrami geometrii układu kierowniczego
        /// @param initial_state Struktura z początkowym stanem układu kierowniczego
        /// @throws std::invalid_argument Jeśli parametry są nieprawidłowe
        ///
        /// Konstruktor inicjalizuje parametry geometrii układu kierowniczego oraz stan początkowy.
        /// W przypadku nieprawidłowych parametrów, zgłasza wyjątek std::invalid_argument.
        ///
        /// @note Zakłada, że wszystkie parametry są dodatnie i sensowne.

        SteerGeometry(const steer_geometry_params& params, const steer_geometry_state& initial_state = steer_geometry_state{0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0}) {};
            





        //////***** Gettery /////////////////////////

        inline steer_geometry_state get_state() const { return state_; }


        inline slip_angles get_slip_angles() const {
            return {state_.front_left_slip_angle, state_.front_right_slip_angle, state_.rear_left_slip_angle, state_.rear_right_slip_angle};
        }


        inline double get_front_left_steer_angle() const {
            return state_.front_left_steer_angle;
        }

        inline double get_front_right_steer_angle() const {
            return state_.front_right_steer_angle;
        }

        inline double get_steer_angle_on_column() const {
            return state_.steer_angle_on_column;
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

        inline steer_angles get_steer_angels() const {

            return  { state_.front_left_steer_angle, state_.front_right_steer_angle, state_.steer_angle_on_column };
        }

        inline  slip_angles get_slip_angels() {


            return  { state_.front_left_slip_angle, state_.front_right_slip_angle, state_.rear_left_slip_angle, state_.rear_right_slip_angle };
        }

        //////////////////////////////////////////////



        //////***** Settery /////////////////////////


        inline void set_state(const steer_geometry_state& new_state) {
            state_ = new_state;
        }


        inline void set_slip_angles(const slip_angles& new_slip_angles) {
            state_.front_left_slip_angle = new_slip_angles.front_left_slip_angle;
            state_.front_right_slip_angle = new_slip_angles.front_right_slip_angle;
            state_.rear_left_slip_angle = new_slip_angles.rear_left_slip_angle;
            state_.rear_right_slip_angle = new_slip_angles.rear_right_slip_angle;
        }

        inline void set_steer_angles(const steer_angles& new_steer_angles) {
            state_.front_left_steer_angle = new_steer_angles.front_left_steer_angle;
            state_.front_right_steer_angle = new_steer_angles.front_right_steer_angle;
            state_.steer_actual = new_steer_angles.steer_actual;
        }



        inline void set_front_left_steer_angle(double angle) {
            state_.front_left_steer_angle = angle;
        }

        inline void set_front_right_steer_angle(double angle) {
            state_.front_right_steer_angle = angle;
        }

        inline void set_steer_on_column(double angle) {
            state_.steer_angle_on_column = angle;
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

        /// This function converts the current state of the steer geometry to a vector representation.
        /// @return A vector containing the current state of the steer geometry.
        ///

        std::vector<double> state_to_vector() const {} ;

        /// This function converts a vector representation to the current state of the steer geometry.
        /// @param vec A vector containing the state of the steer geometry.
        /// @throws std::invalid_argument If the vector size is not equal to 7.

        void vector_to_state(const std::vector<double>& vec) {};


        //////////////////////////////////////////////


        ///***************///// Calculate slip angles ///////*****************///
        /// This function calculates the slip angles for each wheel based on the vehicle's velocity and yaw rate.
        /// It assumes a simple kinematic model for the vehicle.
        ///
        /// @param vx Vehicle's longitudinal velocity
        /// @param vy Vehicle's lateral velocity
        /// @param yaw_rate Vehicle's yaw rate
        /// @return void
        ///


        // returning function calculate slip angles based on vehicle's velocity and yaw rate
        slip_angles calculate_slip_angles( double vx = 0.0, double vy = 0.0, double yaw_rate = 0.0) const {};

        /// returning  function calculates the steer angles based on the vehicle's geometry and the actual  steering on steering column.
        steer_angles calculate_steer_angles() const {} ;



        //  void non reutirng, changing class elements function that  Calculate steer angles based on link geometry -> for now assuming an ideal ackerman steering geometry
         void calculate_and_set_steer_angles () {};

         
        /// This function calculates the derivative of the steer angle based on the input steer angle.
         void calculate_and_set_slip_angles(double vx = 0.0, double vy = 0.0, double yaw_rate = 0.0) {};




        




        

        

        void calculate_steer_angles( ) const  {
             
            // for now assuming an ideal ackerman steering geometry
            


        }






        double derivative_steer(double steer_input) const {
            
            // 1 rzędowy układ kierowniczy
            double derivative = (steer_input - state_.steer_actual) / params_.T;

            // TODO : Jakieś filtry górno/dolno przepustowe ? 

            return derivative;
        }
        
        void update(double steer_input, double dt) {
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