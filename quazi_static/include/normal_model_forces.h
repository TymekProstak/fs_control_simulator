#ifndef NORMAL_MODEL_FORCES_H
#define NORMAL_MODEL_FORCES_H

#include "libs.h"
#include <vector>
#include <stdexcept>

namespace metzler_model {

    ///***************///// PARAMS STRUKTURA ///////*****************///
    /**
     * @brief Struktura parametrów modelu sił normalnych działających na koła pojazdu.
     * 
     * @param drag_coefficient Współczynnik oporu powietrza (aerodynamic drag)
     * @param downforce_coefficient Współczynnik docisku aerodynamicznego
     * @param pressure_center_height Wysokość środka nacisku aerodynamicznego
     * @param wheelbase Rozstaw osi pojazdu
     * @param wheel_distance_front Rozstaw kół przednich
     * @param wheel_distance_rear Rozstaw kół tylnych
     * @param wheel_distance_avg Średni rozstaw kół
     * @param mass Masa pojazdu
     * @param h_cg Wysokość środka ciężkości
     * @param lf Odległość środka ciężkości od przedniej osi
     * @param lr Odległość środka ciężkości od tylnej osi
     * @param gravity Przyspieszenie ziemskie (domyślnie 9.81)
     * @param h_roll_f Wysokość środka obrotu przedniej osi (do obliczeń przechyłu)
     * @param h_roll_r Wysokość środka obrotu tylnej osi (do obliczeń przechyłu)
     * @param front_roll_stiffness Sztywność przechyłu przedniej osi
     * @param rear_roll_stiffness Sztywność przechyłu tylnej osi
     */
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
        double gravity ;
        double h_roll_f;
        double h_roll_r;
        double front_roll_stiffness;
        double rear_roll_stiffness;
    };

    ///***************///// FORCES STRUKTURA ///////*****************///
    /**
     * @brief Struktura przechowująca wartości sił normalnych na poszczególnych kołach.
     * 
     * @param front_left_force Siła normalna na przednim lewym kole
     * @param front_right_force Siła normalna na przednim prawym kole
     * @param rear_left_force Siła normalna na tylnym lewym kole
     * @param rear_right_force Siła normalna na tylnym prawym kole
     */
    struct normal_model_forces_output {
        double front_left_force;
        double front_right_force;
        double rear_left_force;
        double rear_right_force;
    };

    ///***************///// KLASA NORMALMODEL ///////*****************///
    /**
     * @brief Klasa modelująca rozkład sił normalnych na kołach pojazdu.
     * 
     * Pozwala na obliczenie sił normalnych na podstawie parametrów pojazdu oraz aktualnych przyspieszeń.
     */
    class NormalModelForces {
    public:
        /**
         * @brief Konstruktor klasy NormalModelForces
         * @param params Struktura z parametrami modelu sił normalnych
         */
        NormalModelForces(const normal_model_forces_params& params);

        /**
         * @brief Oblicza siły normalne na kołach pojazdu.
         * 
         * @param vx Prędkość wzdłużna pojazdu [m/s] w Inertial Frame
         * @param vy Prędkość poprzeczna pojazdu [m/s] w Inertial Frame
         * @param ax Przyspieszenie wzdłużne pojazdu [m/s^2] w Inertial Frame
         * @param ay Przyspieszenie poprzeczne pojazdu [m/s^2] w Inertial Frame
         * @return Struktura z siłami normalnymi na kołach
         */
        normal_model_forces_output calculate_normal_forces(double vx = 0.0, double vy = 0.0, double ax = 0.0, double ay = 0.0) const;



        void calculate_and_set_normal_forces(double vx = 0.0, double vy = 0.0, double ax = 0.0, double ay = 0.0) ;

        normal_model_forces_output calculate_set_and_return_normal_forces(double vx = 0.0, double vy = 0.0, double ax = 0.0, double ay = 0.0) ;
            
        
        /**
         * @brief Zwraca aktualne parametry modelu sił normalnych.
         */
        inline normal_model_forces_params get_params() const {
            return params_;
        }

        /**
         * @brief Zwraca aktualne siły normalne na kołach.
         */
        inline normal_model_forces_output get_forces() const {
            return forces_;
        }

        /**
         * @brief Zwraca siłę normalną na przednim lewym kole.
         */
        inline double get_left_front_force() const {
            return forces_.front_left_force;
        }
        /**
         * @brief Zwraca siłę normalną na przednim prawym kole.
         */
        inline double get_right_front_force() const {
            return forces_.front_right_force;
        }
        /**
         * @brief Zwraca siłę normalną na tylnym lewym kole.
         */
        inline double get_left_rear_force() const {
            return forces_.rear_left_force;
        }
        /**
         * @brief Zwraca siłę normalną na tylnym prawym kole.
         */
        inline double get_right_rear_force() const {
            return forces_.rear_right_force;
        }

        /**
         * @brief Ustawia nowe parametry modelu sił normalnych.
         * @param new_params Nowe parametry
         */
        inline void set_params(const normal_model_forces_params& new_params) {
            params_ = new_params;
        }

        /**
         * @brief Ustawia nowe siły normalne na kołach.
         * @param new_forces Nowe siły
         */
        inline void set_forces(const normal_model_forces_output& new_forces) {
            forces_ = new_forces;
        }

        /**
         * @brief Ustawia siłę normalną na przednim lewym kole.
         * @param force Nowa siła
         */
        inline void set_left_front_force(double force) {
            forces_.front_left_force = force;
        }
        /**
         * @brief Ustawia siłę normalną na przednim prawym kole.
         * @param force Nowa siła
         */
        inline void set_right_front_force(double force) {
            forces_.front_right_force = force;
        }
        /**
         * @brief Ustawia siłę normalną na tylnym lewym kole.
         * @param force Nowa siła
         */
        inline void set_left_rear_force(double force) {
            forces_.rear_left_force = force;
        }
        /**
         * @brief Ustawia siłę normalną na tylnym prawym kole.
         * @param force Nowa siła
         */
        inline void set_right_rear_force(double force) {
            forces_.rear_right_force = force;
        }

        /**
         * @brief Zwraca siły normalne jako wektor [FL, FR, RL, RR].
         */
        inline std::vector<double> forces_to_vector() const {
            return {forces_.front_left_force, forces_.front_right_force, forces_.rear_left_force, forces_.rear_right_force};
        }

        /**
         * @brief Ustawia siły normalne na podstawie wektora [FL, FR, RL, RR].
         * @param forces_vector Wektor sił (musi mieć 4 elementy)
         * @throws std::invalid_argument jeśli rozmiar wektora jest różny od 4
         */
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
        normal_model_forces_params params_; ///< Parametry modelu sił normalnych
        normal_model_forces_output forces_;        ///< Aktualne siły normalne na kołach
    };

} // namespace metzler_model

#endif // NORMAL_MODEL_FORCES_H