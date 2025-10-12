#include "double_track.hpp"

namespace lem_dynamics_sim{

    State model_derative(const ParamBank& P, const  State& x, const Input& u) {

        double areo_drag = P.get("Cd") * x.vx * x.vx;
        double areo_downforce_front = P.get("Cl1") * x.vx * x.vx;
        double areo_downforce_rear  = P.get("Cl2") * x.vx * x.vx;
        double areo_downforce = areo_downforce_front + areo_downforce_rear;
        double rolling_resistance = P.get("Cr") * ( P.get("m") * P.get("g") + areo_downforce ) ;

        double Fx_total = x.fx_fl * cos(x.delta_left) - x.fy_fl * sin(x.delta_left) + x.fx_fr * cos(x.delta_right) -  x.fy_fr * sin(x.delta_right) + x.fx_rl + x.fx_rr - rolling_resistance - areo_drag;
        double Fy_total = x.fy_fl * cos(x.delta_left) + x.fx_fl * sin(x.delta_left) + x.fy_fr * cos(x.delta_right) + x.fx_fr * sin(x.delta_right) + x.fy_rl + x.fy_rr;
        double Mz = ;
        State temp;
        temp.setZero();

        // kinematics + dynamics of CoG and vehicle body
        temp.x = x.vx * cos(x.yaw) - x.vy * sin(x.yaw);
        temp.y = x.vy * cos(x.yaw) + x.vx * sin(x.yaw);
        temp.yaw = x.yaw_rate;
        temp.vx = Fx_total / P.get("m") + x.vy * x.yaw_rate;
        temp.vy = Fy_total / P.get("m") - x.vx * x.yaw_rate;
        temp.yaw_rate = Mz / P.get("Iz");


        // settin prev_ax and prev_ay for mass transfer model
        temp.prev_ax = (Fx_total / P.get("m") - x.prev_ax)/P.get("simulation_time_step") ;
        temp.prev_ay = ( Fy_total / P.get("m") - x.prev_ay)/P.get("simulation_time_step") ;



        temp += derative_steering(P,x,u);
        temp += derative_drivetrain(P,x,u);
        temp += derative_tire_model(P,x,u);
        temp += derative_wheels_dynamics_model(P,x,u);
        return temp;

    }

    struct Log_Info{


        double kappa_fl;
        double kappa_fr;
        double kappa_rl;
        double kappa_rr;

        double slip_angle_fl;
        double slip_angle_fr;
        double slip_angle_rl;
        double slip_angle_rr;

        double slip_angle_body;

        double fz_fl;
        double fz_fr;
        double fz_rl;
        double fz_rr;

        double fy_fl;
        double fy_fr;
        double fy_rl;
        double fy_rr;

        double fx_fl;
        double fx_fr;
        double fx_rl;
        double fx_rr;

        double kappa_fl;
        double kappa_fr;
        double kappa_rl;
        double kappa_rr;

        double torque;
        double torque_left;
        double torque_right;

        double omega_rl;
        double omega_rr;

        double delta_left;
        double delta_rigth;
        double rack_angle;

        double ax;
        double ay;

        double yaw_rate;
        double vx;
        double vy;
        double x;
        double y;
        double yaw;

        double Power_total;

        double rack_angle_request;
        double torque_request;

        double time;

        double total_drag;
        double total_downforce;
        

    }

    // sending log info calculated in simplified way

    Log_Info log_info(const State& x, const Input& u, const ParamBank& P , int step_number){

        Log_Info info;


        double vx = std::sqrt(x.vx * x.vx + P.get("epsilon") * P.get("epsilon")); // to avoid division by zero

        
        info.kappa_fl = 0.0 ; // przednie koła są beznapędowe
        info.kappa_fr = 0.0; // przednie koła są beznapędowe
        info.kappa_rl = x.omega_rl * P.get("R") / vx - 1;
        info.kappa_rr = x.omega_rr * P.get("R") / vx - 1;

        info.slip_angle_fl = x.slip_angle_fl;
        info.slip_angle_fr = x.slip_angle_fr;
        info.slip_angle_rl = x.slip_angle_rl;
        info.slip_angle_rr = x.slip_angle_rr;

        info.slip_angle_body = std::atan2(x.vy, vx);

        info.fz_fl = 
        info.fz_fr = 
        info.fz_rl = 
        info.fz_rr = 

        info.fy_fl = x.fy_fl;
        info.fy_fr = x.fy_fr;
        info.fy_rl = x.fy_rl;
        info.fy_rr = x.fy_rr;

        info.fx_fl =  0.0; // przednie koła są beznapędowe
        info.fx_fr = 0.0; // przednie koła są beznapędowe
        info.fx_rl = x.fx_rl;
        info.fx_rr = x.fx_rr;

        info.Power_total = (x.torque) * (x.omega_rr + x.omega_rl)/2 ;

       

        info.torque = x.torque;
        info.torque_left = x.torque_left;
        info.torque_right = x.torque_right;

        info.omega_rl = x.omega_rl;
        info.omega_rr = x.omega_rr;

        info.delta_left = x.delta_left;
        info.delta_rigth = x.delta_right;
        info.rack_angle = x.rack_angle;

        info.ax = () / P.get("m");
        info.ay =  () / P.get("m");

        info.yaw_rate = x.yaw_rate;
        info.vx = x.vx;
        info.vy = x.vy

        info.time = step_number * P.get("simulation_time_step");

        info.rack_angle_request = u.rack_angle_request;
        info.torque_request = u.torque_request;
        info.x = x.x;
        info.y = x.y;
        info.yaw = x.yaw;
        info.total_drag = P.get("Cd") * x.vx * x.vx + P.get("Cr") * ( P.get("m") * P.get("g") + P.get("Cl1") * x.vx * x.vx + P.get("Cl2") * x.vx * x.vx ) ;
        info.total_downforce = P.get("Cl1") * x.vx * x.vx + P.get("Cl2") * x.vx * x.vx;

        return info;
}