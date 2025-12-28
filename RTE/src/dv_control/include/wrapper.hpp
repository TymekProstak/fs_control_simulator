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

#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include<cmath>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "stanley.hpp"
#include "pure_pursuit.hpp"
#include "dv_interfaces/Control.h"
#include "ParamBank.hpp"
#include "path_processing.hpp"
#include "utilities.hpp"
#include "mpc_interface.hpp"
#include "dv_interfaces/MPCDebug.h"

#include <unsupported/Eigen/MatrixFunctions> 
namespace v2_control {

    using json = nlohmann::json;


    class Controller {
    public:
    	Controller();
        Controller(ros::NodeHandle &nh, const ParamBank &param);
        void pathCallback(const dv_interfaces::Path &msg);
        void odometryCallback(const nav_msgs::Odometry& msg);
        // void rack_sensorCallback(const dv_interfaces::RackSensor& msg);

    private:
    
        ros::Subscriber path_sub;
        ros::Subscriber odom_sub;
        // ros::Subscriber rack_sensor_sub;
        
        ros::Publisher pub_control;
        ros::Publisher pub_geo_marker;
        ros::Publisher pub_ref_path;
        ros::Publisher pub_debug_info;
        ros::Publisher pub_mpc_debug;

        PathProcessResult ref_path;
       
        State current_state;
        ParamBank param_;
        
        void getCurrentState (const nav_msgs::Odometry& msg);
        void publishControlCommand (double steering_angle,double torque_request);
        void publishLookaheadMarker (const geo_control_return &control_output);
        void publishReferencePath(const Eigen::VectorXd &X,const Eigen::VectorXd &Y);
        void geometricControl();

        void convert_state_to_mpc_state();

        void ZOH_for_steering();
        double acc_to_throttle_percentage(double a_desired);


        void sendDebugInfo();

        bool eligible_path = false;

        

        Stanley stanley;
        Pure_Pursuit pure_pursit;

        Eigen::VectorXd X_last_from_pp;
        Eigen::VectorXd Y_last_from_pp;

        MPC_State mpc_state;
        double last_u_opt_from_mpc = 0.0;

        MPCInterface mpc;

        
    
        Eigen::Matrix2d Ad_maxon;
        Eigen::Vector2d Bd_maxon;
        
        double ey_sum = 0.0;
        int ey_count = 0;
        double epsi_sum = 0.0;

        bool first_path_received = false;

        bool all_path_recived = false;

        // Legacy "full path" buffers (were read from msg.X_path_all/msg.Y_path_all, which do not exist in dv_interfaces/Path)
        Eigen::VectorXd X_path_all;
        Eigen::VectorXd Y_path_all;
        double path_yaw_ = 0.0;

	 
    };
}


