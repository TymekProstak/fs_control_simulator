#pragma once

#include "double_track.hpp"
#include "cone_detector.hpp"
#include "cone_track.hpp"
#include "pid.hpp"
#include "kalman.hpp"

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "dv_interfaces/Control.h"
#include "dv_interfaces/Cone.h"
#include "dv_interfaces/Cones.h"
#include "dv_interfaces/full_state.h"


#include <string>
#include <deque>
#include <optional>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <random>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dv_interfaces/MPCDebug.h>
//// ===========================================================================
///  Symulator LEM Dynamics + ROS
///  - obsługa wejść sterujących
///  - kroki symulacji i publikacje danych
///  - modelowanie opóźnień czujników (kamera / INS / enkoder)
//// ===========================================================================

namespace lem_dynamics_sim_ {

    

//// ===========================================================================
//  Helpery do kolorowania markerów stożków
static std_msgs::ColorRGBA color_from_class_gt(const std::string& cls, float alpha = 0.95f)
{
    std_msgs::ColorRGBA c; 
    c.a = alpha;

    if (cls == "yellow" || cls == "Y") {
        c.r = 1.0f; c.g = 0.9f; c.b = 0.0f;
    }
    else if (cls == "blue" || cls == "B") {
        c.r = 0.1f; c.g = 0.3f; c.b = 1.0f;
    }
    else if (cls == "orange" || cls == "O") {
        c.r = 1.0f; c.g = 0.4f; c.b = 0.0f;
    }
    else {
        c.r = 0.6f; c.g = 0.6f; c.b = 0.6f;
    }
   
    return c;
}

static std_msgs::ColorRGBA color_from_class_vision(const std::string& cls, float alpha = 0.95f)
{
    std_msgs::ColorRGBA c;
    c.a = alpha;

    if (cls == "yellow")      { c.r = 1.0f; c.g = 1.0f; c.b = 0.0f; }      // czysty żółty
    else if (cls == "blue")   { c.r = 0.0f; c.g = 0.3f; c.b = 1.0f; }      // nasycony niebieski
    else if (cls == "orange") { c.r = 1.0f; c.g = 0.55f; c.b = 0.0f; }     // żywy pomarańcz
    else                      { c.r = 0.7f; c.g = 0.7f; c.b = 0.7f; }      // neutralny szary

    return c;
}


//// ===========================================================================

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
    m.type = visualization_msgs::Marker::CYLINDER; // stożek jako cylinder
    m.action = visualization_msgs::Marker::ADD;

    // --- pozycja i orientacja ---
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z + 0.18;  // podniesienie o połowę wysokości
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    // --- rozmiar stożka ---
    m.scale.x = 0.28;
    m.scale.y = 0.28;
    m.scale.z = 0.36;

    // --- kolor i czas życia ---
    m.color = col;
    m.lifetime = lifetime;
    m.frame_locked = false; // nie przypinaj do TF – Foxglove sam zaktualizuje

    return m;
}

//// ===========================================================================
//  Struktury pomocnicze
//// ===========================================================================
struct INS_data {
    double x{};
    double y{};
    double yaw{};
    double vx{};
    double vy{};
    double yaw_rate{};
};


//// ===========================================================================
//  Klasa główna symulatora ROS
//// ===========================================================================
class Simulation_lem_ros_node {

    // ===== tick phases (0..period-1) =====
    int phase_camera_shoot_          = 0;
    int phase_wheel_encoder_reading_ = 0;
    int phase_ins_reading_           = 0;
    int phase_torque_apply_          = 0;
    int phase_steer_apply_           = 0;
    int phase_gps_speed_reading_     = 0;
    int phase_control_input_read_    = 0;

// RNG do losowania faz (stałe na start symulacji)
std::mt19937 phase_rng_{std::random_device{}()};

// helper do losowania fazy
int pick_phase_(int period);


public:
    Simulation_lem_ros_node(ros::NodeHandle& nh,
                            const std::string& param_file,
                            const std::string& track_file,
                            const std::string& log_file);

    ~Simulation_lem_ros_node();

    // ====== Główna pętla ======
    void step();

    // ====== Dostęp do stanu ======
    State get_state() { return state_; }
    ParamBank get_parameters() { return P_; }
    int get_step_number() { return step_number_; }

    // ====== ROS callback ======
    void dv_control_callback(const dv_interfaces::Control::ConstPtr& msg);
private:
    //// =======================================================================
    //  Logowanie danych do csv nt metryk jazdy
    //// =======================================================================
    std::string metrics_log_file_path_;
    void log_metric_of_ride_data_();
    double ey_avg_; // average lateral error along the track
    double epsi_avg_; // average heading error along the track
    double vs_avg_;  // average speed along the track
    std::vector<double> ten_biggest_slip_ratio_;
    std::vector<double> ten_biggest_beta_angle_;
    std::vector<double> ten_biggest_ey_;
    std::vector<double> ten_biggest_epsi_;
    double percetage_of_time_beta_over_9_;
    double percetage_of_time_tc_active_;
    double time_tc_active_ = 0.0;
    double time_beta_over_9_ = 0.0;
    double kappa_metric_ = 0.0;
    double slip_angle_metric_ = 0.0;
    double slip_body_metric_ = 0.0;

    // ---- crash handling ----
    bool crashed_ = false;
    std::string crash_reason_;
    int crash_step_ = -1;
    double crash_time_s_ = -1.0;

    // opcjonalnie: żeby nie spamować shutdownem
    bool shutdown_requested_ = false;

    inline void mark_crash_(const std::string& reason)
    {
        if (crashed_) return;
        crashed_ = true;
        crash_reason_ = reason;
        crash_step_ = step_number_;
        crash_time_s_ = step_number_ * P_.get("simulation_time_step");
    }



    ros::Subscriber sub_mpc_debug_;

    void mpc_debug_callback_(const dv_interfaces::MPCDebug::ConstPtr& msg);

    //// =======================================================================
    //  ROS
    //// =======================================================================
    ros::Subscriber sub_control_;
    ros::Publisher  pub_ins_;
    ros::Publisher  pub_cones_;
    ros::Publisher  pub_markers_cones_gt_;
    ros::Publisher  pub_markers_cones_vis_;
    ros::Publisher  pub_pose_true_;
    ros::Publisher  pub_pose_ins_;
    ros::Publisher  pub_log_full_;
    ros::Publisher  pub_marker_bolid_;
    ros::Publisher  pub_gg_sphere_marker_;
    tf2_ros::TransformBroadcaster tf_br_;

    //// =======================================================================
    //  Parametry, stany i sterowanie
    //// =======================================================================
    ParamBank P_;
    State     state_;
    Track     track_global_;

    PIDController traction_control_pid_drive_;
    PIDController traction_control_pid_brake_;


    double wheel_speed_read_right = 0.0; // ostatnia odczytana prędkość z enkodera kół na prawym kole (w m/s)
    double wheel_speed_read_left = 0.0; // ostatnia odczytana prędkość z enkodera kół na prawym kole (w m/s)

    double prev_I_speed_pid = 0.0; // poprzednia wartość całki w PID prędkości
    double prev_error_speed_pid = 0.0; // poprzedni błąd w PID prędkości

    double torque_command_to_invert_ = 0.0; // actual torque sent to tractive system
    double steer_command_to_maxon_ = 0.0; // actual steer sent to steering actuator

    dv_interfaces::Control last_input_cached; // last input on control topic, cached for dv board reading
    dv_interfaces::Control  last_input_read_by_dv_board; // last input read by dv board (at fixed cadence)

     //// =======================================================================
    //  INS, kolejki kamer, dane
    //// =======================================================================
    Track    track_to_be_published_;
    INS_data ins_data_to_be_published_;
    INS_data last_ins_data_already_published_;

    int step_number_ = 0;

    int step_of_camera_shoot_          = 0;
    int step_of_wheel_encoder_reading_ = 0;
    int step_of_ins_reading_           = 0;
    int step_gps_speed_reading_        = 0;
    // DV_BOARD_READ cadence (board czyta wejście z ROS)
    int step_of_control_input_read_ = 0;

    // board -> steering actuator cadence (board wysyła steering)
    int step_of_steer_input_sending_ = 0;

    // board -> tractive system cadence (board wysyła torque) + na tym samym ticku liczę PID
    int step_number_torque_input_sending_ = 0;
    int step_imu_reading_           = 0;
    
    int last_frame_size_ = 0;


    GpsMeasurement last_gps_{};
    bool has_last_gps_ = false;
    ImuMeasurement last_imu_{}; 
    bool has_last_imu_ = false;
    double sim_b_g  = 0.0; // gyro bias
    double sim_b_ax = 0.0; // acc x bias
    double sim_b_ay = 0.0; // acc y bias
    std::string ins_mode_ = "gauss"; // "gauss" or "kalman" -> tryb ins
    bool lov_level_control_on = false;
    int sim_time = -1; // domyślnie nieskończony czas symulacji
    KalmanFilter kalman_filter_;

    

    struct CameraTask {
        int   ready_step;
        Track frame;
    };
    std::deque<CameraTask> camera_queue_;
    std::deque<ros::Time>  timestamp_queue_;

    //// =======================================================================
    //  Pomocnicze funkcje symulacyjne
    //// =======================================================================

    // helpers do losowania opóźnień
    double random_noise_generator_() const;
    void compute_step_intervals_from_params_();
    double sample_vision_exec_time_() const;

    // publisery rosowe do wizualizacji i danych
    void publish_ins_(const INS_data& ins);
    void publish_cones_(const Track& cones, ros::Time timestamp);
    void publish_cones_vision_markers_(const Track& det, const ros::Time& acquisition_stamp);
    void publish_cones_gt_markers_();
    void publish_ready_camera_frames_from_queue_();

    // czytanie wejść z kontroli i z czujników
    void read_wheel_encoder_if_due_();
    void read_ins_if_due_();
    void read_control_by_dv_board_if_due();
    void read_steer_by_orin_if_due_();

    // aplikacja z dv_board do aktuatorów i obrót wizji 
    void shoot_camera_or_enqueue_if_due_();
    void send_to_ts_if_due();

    // Kompatybilność: w step() jest wywołanie send_steer_to_maxon_if_due_() -> dlatego że obecnie od razu jak orin czyta to wysła do
    inline void send_steer_to_maxon_if_due_() { read_steer_by_orin_if_due_(); }

    // publikacja TF i markerów bolidu
    void publish_bolid_tf_ins(const INS_data& ins);
    void publish_bolid_tf_true();
    void pub_full_state_();
    void publish_bolid_marker_();

    
};

} // namespace lem_dynamics_sim_
