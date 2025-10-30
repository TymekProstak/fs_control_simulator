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
#include <mutex>
#include <thread>
#include <queue>
#include <atomic>
#include <condition_variable>
#include <algorithm>
#include <cmath>
#include <random>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>

//// ===========================================================================
///  Symulator LEM Dynamics + ROS
///  - obsługa wejść sterujących
///  - kroki symulacji i publikacje danych
///  - modelowanie opóźnień czujników (kamera / INS / enkoder)
///  - asynchroniczny logger (nie blokuje pętli fizyki)
//// ===========================================================================

namespace lem_dynamics_sim_ {

//// ===========================================================================
//  Helpery do kolorowania markerów stożków
//// ===========================================================================
static std_msgs::ColorRGBA color_from_class(const std::string& cls, float alpha = 0.95f)
{
    std_msgs::ColorRGBA c; 
    c.a = alpha;
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

struct DV_control_input {
    double torque{};
    double steer{};
    int torque_mode{}; // 0 - torque , 1 - speed
};

//// ===========================================================================
//  Klasa główna symulatora ROS
//// ===========================================================================
class Simulation_lem_ros_node {
public:
    Simulation_lem_ros_node(ros::NodeHandle& nh,
                            const std::string& param_file,
                            const std::string& track_file,
                            const std::string& log_file);

    ~Simulation_lem_ros_node();

    // ====== Główna pętla ======
    void step();

    // ====== Dostęp do stanu ======
    State get_state() const;
    ParamBank get_parameters() const;
    int get_step_number() const;

    // ====== ROS callback ======
    void dv_control_callback(const dv_interfaces::Control::ConstPtr& msg);
private:
    //// =======================================================================
    //  Logowanie równoległe (asynchroniczne)
    //// =======================================================================
    std::ofstream log_file_;
    bool logging_enabled_ = false;
    // kolejka i wątek loggera
    std::queue<std::string> log_queue_;
    std::mutex log_mutex_;
    std::condition_variable log_cv_;
    std::thread log_thread_;
    std::atomic<bool> logging_thread_running_{false};

    void start_logging_thread_();
    void stop_logging_thread_();
    void log_state_(); // wrzuca do kolejki

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
    tf2_ros::TransformBroadcaster tf_br_;

    //// =======================================================================
    //  Parametry, stany i sterowanie
    //// =======================================================================
    ParamBank P_;
    State     state_;
    Track     track_global_;

    double pid_prev_I_          = 0.0;
    double pid_prev_error_      = 0.0;
    double target_wheel_speed_  = 0.0;
    double pid_speed_out_       = 0.0;
    double pid_omega_actual_    = 0.0;
    double torque_command_      = 0.0;
    double torque_command_to_invert_ = 0.0;
    double steer_command_       = 0.0;

    int torque_mode_ = 0; // 0 - torque, 1 - speed
    DV_control_input last_input_requested_{};

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
    int step_of_torque_input_application_ = 0;
    int step_of_steer_input_application_  = 0;
    int step_number_pid_update_period_    = 0;

    

    struct CameraTask {
        int   ready_step;
        Track frame;
    };
    std::deque<CameraTask> camera_queue_;
    std::deque<ros::Time>  timestamp_queue_;

    //// =======================================================================
    //  Pomocnicze funkcje symulacyjne
    //// =======================================================================
    double random_noise_generator_() const;
    void publish_ins_(const INS_data& ins);
    void publish_cones_(const Track& cones, ros::Time timestamp);
    void publish_cones_vision_markers_(const Track& det, const ros::Time& acquisition_stamp);
    void publish_cones_gt_markers_();
    void compute_step_intervals_from_params_();
    void read_wheel_encoder_if_due_();
    void read_ins_if_due_();
    void shoot_camera_or_enqueue_if_due_();
    void publish_ready_camera_frames_from_queue_();
    void update_pid_if_due_();
    void apply_delayed_inputs_if_due_();
    double sample_vision_exec_time_() const;
    void publish_bolid_tf_ins(const INS_data& ins);
    void publish_bolid_tf_true();
    void pub_full_state_();
};

} // namespace lem_dynamics_sim_
