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
        Simulation_lem_ros_node(std::string param_file, std::string track_file, str::string log_file);

        void step(const Input& u);
        State get_state() const;
        Track get_visible_track() const;
        ParamBank get_parameters() const;
        Input get_last_input() const;
        INS_data get_last_ins_data() const;
        int get_step_number() const;



    private:
        ParamBank P_;
        State state_;
        Track track_global_;
        // waiting - so  to be published data as we try to model  =systens latency/delats
        Track track_visible_to_be_published_;
        Input input_to_be_published;
        INS_data ins_data_to_be_published_;
        
        
        // last time published data
        Track last_visible_track_;
        Input last_input_;
        INS_data last_ins_data_;

        // simulation step counter
        int step_number_ = 0;

    };


    //  system initialization and class constructor
    Simulation_lem_ros_node::Simulation_lem_ros_node(std::string param_file, std::string track_file) {
        P_.loadFromFile(param_file);
        state_.setZero();
        track_global_ = load_track_from_file(track_file);
    }








}





