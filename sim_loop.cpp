#pragma once

#include "double_track.hpp"
#include "cone_detector.hpp"
#include "cone_track.hpp"
#include "<ros/ros.h>"
#include "<string>"
#
namespace lem_dynamics_sim_{    


    struct INS_data {
        double x;
        double y;
        double yaw;
        double vx;
        double vy;
        double yaw_rate;
    };

    class Simulation_lem_ros_node {
    public:
        Simulation_lem_ros_node(ros::NodeHandle &nh,std::string param_file, std::string track_file, str::string log_file);


        // step symulacji o jeden krok -> główny kod
        void step(const Input& u);
        State get_state() const;
        Track get_visible_track() const;
        ParamBank get_parameters() const;
        Input get_last_input() const;
        INS_data get_last_ins_data() const;
        int get_step_number() const;
        // ROS callbacks
        void dv_control_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        



    private:

        ros::Subscriber sub_control;
        ros::Publisher pub_ins;
        ros::Publisher pub_cones

        ParamBank P_;
        State state_;
        Track track_global_;
        // waiting - so  to be published data as we try to model  =systens latency/delats
        Track last_visible_track_from_cone_detector;
        Input last_input_from_dv_control;;
        INS_data last_ins_data;
        double pid_prev_I;
        double pid_prev_omega;
        
        
        // data to be published data
        Track track_to_be_published;
        Input input_now_requested_;
        INS_data ins_data_to_be_published;

        // simulation step counter
        int step_number_ = 0;
        int step_number_to_read_omega = 0;
        int step_number_to_read_cones = 0;
        int step_number_to_read_ins = 0;
        int step_number_to_apply_steer_input = 0;
        int step_number_to_apply_torque_input = 0;

        int step_numer_to_update_pid = 0;
        int step_number_t

    };


    //  system initialization and class constructor
    Simulation_lem_ros_node::Simulation_lem_ros_node(std::string param_file, std::string track_file) {
        P_.loadFromFile(param_file);
        state_.setZero();
        track_global_ = load_track_from_file(track_file);
        pid_prev_I = 0.0;
        pid_prev_omega = 0.0;

    }








}





