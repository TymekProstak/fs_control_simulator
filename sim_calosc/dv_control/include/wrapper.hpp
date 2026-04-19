#pragma once 

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

// custom msgs
#include <dv_interfaces/Path.h>
#include <dv_interfaces/DV_board.h>
#include <dv_interfaces/Control.h>
#include <dv_interfaces/MPCDebug.h>

#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "stanley.hpp"
#include "pure_pursuit.hpp"
#include "ParamBank.hpp"
#include "path_processing.hpp"
#include "utilities.hpp"
#include "mpc_interface.hpp"
#include "smooth_path_qp.hpp"

#include <unsupported/Eigen/MatrixFunctions> 

namespace v2_control {

    using json = nlohmann::json;

    class Controller {
    public:
        Controller();
        Controller(ros::NodeHandle &nh, const ParamBank &param);

        void pathCallback(const dv_interfaces::Path &msg);
        void odometryCallback(const nav_msgs::Odometry& msg);
        void dvBoardCallback(const dv_interfaces::DV_board::ConstPtr& msg);

    private:
        ros::Subscriber path_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber dv_board_sub;
        
        ros::Publisher pub_control;
        ros::Publisher pub_geo_marker;
        ros::Publisher pub_ref_path;
        ros::Publisher pub_debug_info;
        ros::Publisher pub_mpc_debug;
        ros::Publisher pub_gg_marker;

        PathProcessResult ref_path;
       
        State current_state;
        ParamBank param_;
        
        void getCurrentState(const nav_msgs::Odometry& msg);
        void publishControlCommand(double steering_angle, double torque_request, double mtv);
        void publishLookaheadMarker(const geo_control_return &control_output);
        void publishReferencePath(const Eigen::VectorXd &X, const Eigen::VectorXd &Y);
        void geometricControl();

        void convert_state_to_mpc_state();

        void ZOH_for_steering();
        double acc_to_throttle_percentage(double a_desired);

        void sendDebugInfo();
        void publishGGLimitMarker();

        double updateBangBangMovement(double v_ref_mps, double v_meas_mps);

        bool eligible_path = false;
        
        TrackSpline2D spline_path;
        double last_s0;

        Stanley stanley;
        Pure_Pursuit pure_pursit;

        Eigen::VectorXd X_last_from_pp;
        Eigen::VectorXd Y_last_from_pp;

        MPC_State mpc_state;
        double last_ddelta_opt_from_mpc = 0.0;
        double last_mtv_opt_from_mpc = 0.0;
        double next_target_yaw_rate_from_mpc = 0.0;

        MPCInterface mpc;

        Eigen::Matrix2d Ad_maxon;
        Eigen::Vector2d Bd_maxon;
        
        double ey_sum = 0.0;
        int ey_count = 0;
        double epsi_sum = 0.0;
        double v_s_sum = 0.0;

        double ey_ = 0.0;
        double epsi_ = 0.0;
        double path_yaw_ = 0.0;

        bool first_path_received = false;
        bool all_path_recived = false;

        Eigen::VectorXd X_path_all;
        Eigen::VectorXd Y_path_all;
        double kappa_last_ = 0.0;
        double v_ref_last_ = 0.0;
        double R_ref_last_ = 0.0;
        double alat_ref_last_ = 0.0;
        Vec2 prev_last_point_;
        bool prev_last_point_valid_ = false;

        bool only_low_speed_ = false;

        bool bang_bang_ = false;
        bool has_encoder_speed_ = false;
        double encoder_speed_mps_ = 0.0;
        double bang_bang_last_movement_ = 0.0;
    };
}
