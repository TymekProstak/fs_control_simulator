#pragma once

#include "double_track.hpp"
#include "cone_detector.hpp"
#include "cone_track.hpp"


#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/PoseArray.h>

#include "dv_interfaces/Control.h"
#include "dv_interfaces/Cones.h"


#include <string>
#include <deque>
#include <optional>

#include <algorithm>
#include <cmath>
#include <random>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

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


    static std_msgs::ColorRGBA color_from_class(const std::string& cls, float alpha=0.95f) {
        std_msgs::ColorRGBA c; c.a = alpha;
        if (cls == "yellow")      { c.r = 1.0f; c.g = 0.9f; c.b = 0.0f; }
        else if (cls == "blue")   { c.r = 0.1f; c.g = 0.3f; c.b = 1.0f; }
        else if (cls == "orange") { c.r = 1.0f; c.g = 0.4f; c.b = 0.0f; }
        else                      { c.r = 0.6f; c.g = 0.6f; c.b = 0.6f; }
        return c;
    }
    
    static visualization_msgs::Marker make_cone_marker(
        int id, const std::string& frame,
        double x, double y, double z,
        const std_msgs::ColorRGBA& col,
        const ros::Duration& lifetime)
{
    visualization_msgs::Marker m;
    m.header.frame_id = frame;
    m.header.stamp    = ros::Time::now();
    m.ns   = "cones";
    m.id   = id;
    m.type = visualization_msgs::Marker::CYLINDER; // stożek „wizualny” jako cylinder
    m.action = visualization_msgs::Marker::ADD;

    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z + 0.18; // lekkie podniesienie (wys. ~0.36 m)
    m.pose.orientation.w = 1.0;

    m.scale.x = 0.28;  // średnica
    m.scale.y = 0.28;
    m.scale.z = 0.36;  // wysokość

    m.color = col;
    m.lifetime = lifetime;

    return m;
}


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
                            const std::string& log_file );

    // Główny krok symulacji (wykonaj jeden krok)
    void step();

    
    // Dostęp do stanu i parametrów
    State get_state() const;
    ParamBank get_parameters() const;
    int get_step_number() const;

    // ROS callback (np. sterowanie z zewnątrz)
    void dv_control_callback(const dv::interfaces::Control msg);

private:
    // ==== ROS ====
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


    ros::Publisher pub_markers_cones_gt_; // publikacja conów z toru (ground truth)
    ros::Publisher pub_markers_cones_vis_; // publikacja conów tak jak je widzi kamera (z szumem itp)


    ros::Publisher pub_pose_true_;  // publikacja prawdziwej pozycji coga
    ros::Publisher pub_pose_ins_;   //  publikacja pozycji coga z ins - > ta co ma ramke w sytemie bolide_CoG

    tf2_ros::TransformBroadcaster tf_br_; // do publikacji TF : map -> bolide_CoG , map -> bolide_CoG_true

    int torque_mode_ = 0;  // "torque" lub "speed" - 0 - torque , 1 - speed

    // Ostatnie wejście zewnętrzne (zachowane do opóźnionego zastosowania)
    DV_control_input last_input_requested_{0.0, 0.0,0};

    // Dane do publikacji
    Track    track_to_be_published_;
    INS_data ins_data_to_be_published_;
    // Ostatnie opublikowane dane INS (do obliczenia pochodnych)
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

    // ====== KOLEJKA PRZETWARZANIA KAMERY (modelowanie opóźnienia bo obliczenia wizyjne) ======
    struct CameraTask {
        int   ready_step;  // krok, od którego ramka jest gotowa do publikacji
        Track frame;       // wynik detekcji (np. stożków) w układzie kamery
    };
    std::deque<CameraTask> camera_queue_; // max 3 elementy (ograniczenie niżej)
    std::deque<ros::Time> timestamp_queue_; // max 3 elementy -> czasy zrobienia zdjęć

    // Pomocnicze (los szumu)
    double random_noise_generator_() const;

    // Publikacje (szkielety) — wypełnij wg własnych typów/tematów
    void publish_ins_(const INS_data& ins);
    void publish_cones_(const Track& cones, ros::Time timestamp);

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

    // Pomoc: losowanie czasu obliczeń wizji
    double sample_vision_exec_time_() const; // losuje czas obliczeń wizji [s] z t-Studenta(ν=6)
};

} // namespace lem_dynamics_sim_
