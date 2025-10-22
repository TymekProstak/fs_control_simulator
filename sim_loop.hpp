#pragma once

#include "double_track.hpp"
#include "cone_detector.hpp"
#include "cone_track.hpp"


#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <string>
#include <deque>
#include <optional>

#include <algorithm>
#include <cmath>
#include <random>

/**
 *  Szkielet węzła symulacji LEM + ROS
 *  - obsługa wejść sterujących
 *  - kroki symulacji i publikacje danych
 *  - kolejka przetwarzania ramek kamery (modelowanie opóźnienia obliczeń)
 *
 *  Uwaga: Plik nagłówkowy zawiera tylko deklaracje publiczne i interfejs.
 *  Implementacja w simulation_lem_ros_node.cpp
 */

namespace lem_dynamics_sim_ {

struct INS_data {
    double x{};
    double y{};
    double yaw{};
    double vx{};
    double vy{};
    double yaw_rate{};
};
struct DV_control_input{
    double torque;
    double steer;
    int torque_mode; // 0 - torque , 1 - speed
}

class Simulation_lem_ros_node {
public:
    // Uwaga: poprawiona sygnatura (str::string -> std::string)
    Simulation_lem_ros_node(ros::NodeHandle& nh,
                            const std::string& param_file,
                            const std::string& track_file,
                            const std::string& log_file = std::string());

    // Główny krok symulacji (wykonaj jeden krok)
    void step();

    
    // Dostęp do stanu i parametrów
    State get_state() const;
    ParamBank get_parameters() const;
    int get_step_number() const;

    // ROS callback (np. sterowanie z zewnątrz)
    void dv_control_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);

private:
    // ==== ROS ====
    ros::NodeHandle nh_;
    ros::Subscriber sub_control_;
    ros::Publisher  pub_ins_;
    ros::Publisher  pub_cones_;

    // ==== Model/parametry/stany ====
    ParamBank P_;
    State     state_;
    Track     track_global_;

    // Sterowanie / PID (szkielet)
    double pid_prev_I_      = 0.0;
    double pid_prev_error_  = 0.0;
    double target_wheel_speed_ = 0.0;
    double pid_speed_out_      = 0.0;     // wyjście regulatora prędkości
    double pid_omega_actual_   = 0.0;     // aktualna prędkość kątowa kół (z enkoderów)
    double torque_command_     = 0.0; // may represent torque or speed command depending on mode
    double torque_command_to_invert_ = 0.0;
    double steer_command_      = 0.0;

    std::string torque_mode_ = 0;  // "torque" lub "speed"

    // Ostatnie wejście zewnętrzne (zachowane do opóźnionego zastosowania)
    DV_control_input last_input_requested_{0.0, 0.0,0};

    // Dane do publikacji
    Track    track_to_be_published_;
    INS_data ins_data_to_be_published_;
    INS_data last_ins_data_already_published_;

    // Licznik kroków
    int step_number_ = 0;

    // Interwały czasowe liczone w krokach (ustawiane w ctor po wczytaniu P_)
    int step_of_camera_shoot_          = 0;
    int step_of_wheel_encoder_reading_ = 0;
    int step_of_ins_reading_           = 0;

    int step_of_torque_input_application_ = 0;
    int step_of_steer_input_application_  = 0;

    int step_number_pid_update_period_    = 0;

    // Moment w krokach, kiedy zastosować kolejne wejście (0 = nie czekać)
    int step_number_to_apply_steer_input_  = 0;
    int step_number_to_apply_torque_input_ = 0;

    // ====== KOLEJKA PRZETWARZANIA KAMERY (modelowanie opóźnienia CPU) ======
    struct CameraTask {
        int   ready_step;  // krok, od którego ramka jest gotowa do publikacji
        Track frame;       // wynik detekcji (np. stożków) w układzie kamery
    };
    std::deque<CameraTask> camera_queue_; // max 3 elementy (ograniczenie niżej)

    // Pomocnicze (los szumu)
    double random_noise_generator_() const;

    // Publikacje (szkielety) — wypełnij wg własnych typów/tematów
    void publish_ins_(const INS_data& ins);
    void publish_cones_(const Track& cones);

    // Pomoc: inicjalizacja interwałów krokowych na podstawie P_
    void compute_step_intervals_from_params_();

    // Pomoc: wykonanie jednego „odczytu” enkodera/INS/kamery i aktualizacji PID
    void read_wheel_encoder_if_due_();
    void read_ins_if_due_();
    void shoot_camera_or_enqueue_if_due_();
    void publish_ready_camera_frames_from_queue_();

    void update_pid_if_due_();

    // Pomoc: zastosowanie opóźnionych wejść sterujących
    void apply_delayed_inputs_if_due_();
    double sample_vision_exec_time_() const; // losuje czas obliczeń wizji [s] z t-Studenta(ν=6)
};

} // namespace lem_dynamics_sim_
