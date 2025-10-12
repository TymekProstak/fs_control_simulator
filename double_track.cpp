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

        double slip_angle_fl; // deg
        double slip_angle_fr; // deg
        double slip_angle_rl; // deg
        double slip_angle_rr; // deg

        double slip_angle_body; // deg

        double fz_fl; // N
        double fz_fr; // N
        double fz_rl; // N
        double fz_rr; // N

        double fy_fl; // N
        double fy_fr; // N
        double fy_rl; // N
        double fy_rr; // N

        double fx_fl; // N
        double fx_fr; // N
        double fx_rl; // N
        double fx_rr; // N

        double torque; // Nm
        double torque_left; // Nm
        double torque_right; // Nm

        double omega_rl; // rad/s
        double omega_rr; // rad/s

        double delta_left; // deg
        double delta_rigth; // deg
        double rack_angle; // deg

        double ax; // g units
        double ay; // g units

        double yaw_rate; // rad/s
        double vx; // m/s
        double vy; // m/s
        double x; // m
        double y; // m
        double yaw; // rad

        double Power_total; // kW

        double rack_angle_request; // deg
        double torque_request; // Nm

        double time; // s

        double total_drag; // N
        double total_downforce; // N
        

    }

    // fukncja do liczenia loga przydatnych rzeczy z informacji o stanie pojazdu w danym kroku symulacji


    Log_Info log_info(const State& x, const Input& u, const ParamBank& P , int step_number){

        Log_Info info;


        double m = P.get("m");
        double g = P.get("gravity");
        double w = P.get("w");
        double a = P.get("a");
        double b = P.get("b");
        double t_front = P.get("t_front");
        double t_rear = P.get("t_rear");
        double h = P.get("h");
        double h_roll_f =  P.get("h_roll_f");
        double h_roll_r =  P.get("h_roll_r");

        double Kf = P.get("Kf");
        double Kr = P.get("Kr");
        double K_total =  Kf + Kr;
        double mf = m * a/w;
        double mr = m * b/w;
        double h_prim_f = h - h_roll_f;
        double h_prim_r = h - h_roll_r;


        double r_rear = P.get("r_rear");
        double r_front = P.get("r_front");
        double angle_construction_front = P.get("angle_construction_front");
        double angle_construction_rear = P.get("angle_construction_rear");

        double vx_rr = x.vx + x.yaw_rate * r_rear*std::sin(angle_construction_rear);
        double vy_rr = x.vy - x.yaw_rate * r_front*std::cos(angle_construction_rear);
    
        double vx_rl = x.vy - x.yaw_rate * r_rear*std::sin(angle_construction_rear);
        double vy_rl = x.vy + x.yaw_rate * r_front*std::cos(angle_construction_rear);
    
        double vx_fr = x.vy + x.yaw_rate * r_rear*std::sin(angle_construction_front);
        double vy_fr = x.vy + x.yaw_rate * r_front*std::cos(angle_construction_front);
    
        double vx_fl = x.vy - x.yaw_rate * r_rear*std::sin(angle_construction_front);
        double vy_fl = x.vy + x.yaw_rate * r_front*std::cos(angle_construction_front);
    
        // usuwanie nieregularności przy małych prędkościach w rachunkach slipów - > niefizyczne tylko numeryczne : https://www.amazon.pl/Tire-Vehicle-Dynamics-Hans-Pacejka/dp/0080970168 strona z defincją Magic Fomrula
     
        double vx_rr_denom = std::sqrt(vx_rr*vx_rr + epsilon*epsilon) ;
        double vx_rl_denom = std::sqrt(vx_rl*vx_rl +  epsilon*epsilon);
        double vx_fr_denom = std::sqrt(vx_fr*vx_fr + epsilon*epsilon) ;
        double vx_fl_denom = std::sqrt(vx_fl*vx_fl + epsilon*epsilon) ;

        double slip_angle_fr = state.delta_rigth - std::atan2(vy_fr,vx_fr_denom) ;
        double slip_angle_fl = state.delta_lelft  - std::atan2(vy_fl,vx_fl_denom) ;
        double slip_angle_rr =  - std::atan2(vy_rr,vx_rl_denom);
        double slip_angle_rl =  - std::atan2(vy_rl,vx_rl_denom);

        double slip_ratio_rr = (x.omega_right * R - vx_rr )/vx_rr;
        double slip_ratio_rl = (x.omega_left * R - vx_rl) / vx.vx_rl;
    

        info.kappa_fl = 0.0 ; // przednie koła są beznapędowe
        info.kappa_fr = 0.0; // przednie koła są beznapędowe
        info.kappa_rl = slip_ratio_rl;
        info.kappa_rr = slip_ratio_rr;

        info.slip_angle_fl = slip_angle_fl * 180 / M_PI ;
        info.slip_angle_fr = slip_angle_fr * 180 / M_PI ;
        info.slip_angle_rl =  slip_angle_rl * 180 / M_PI ;
        info.slip_angle_rr = slip_angle_rr * 180 / M_PI ;
     

        info.slip_angle_body = std::atan2(x.vy, vx) * 180 / M_PI ;

        info.fz_fl = 0.5 * mf * g - 0.5 * m * x.ax_prev * h/w  - x.ay_prev/t_front * ( mf * h_roll_f + Kf/K_total *(mf * h_prim_f + mr * h_prim_r)) + 1/2 * P.get("Cl1") * x.vx * x.vx ;
        info.fz_fr =   0.5 * mf * g  - 0.5 * m * x.ax_prev * h/w  + x.ay_prev /t_front * ( mf * h_roll_f + Kf/K_total *(mf * h_prim_f + mr * h_prim_r)) + 1/2 * P.get("Cl1") * x.vx * x.vx ;
        info.fz_rl =   0.5 * mr * g  + 0.5 * m * x.ax_prev * h/w  - x.ay_prev/t_rear * ( mr * h_roll_r + Kr/K_total *(mf * h_prim_f + mr * h_prim_r)) + 1/2 * P.get("Cl2") * x.vx * x.vx ;
        info.fz_rr =  0.5 * mr *g   + 0.5 * m * x.ax_prev * h/w  + x.ay_prev/t_rear * ( mr * h_roll_r + Kr/K_total *(mf * h_prim_f + mr * h_prim_r)) + 1/2 * P.get("Cl2") * x.vx * x.vx ;

        info.fy_fl = x.fy_fl;
        info.fy_fr = x.fy_fr;
        info.fy_rl = x.fy_rl;
        info.fy_rr = x.fy_rr;

        info.fx_fl =  0.0; // przednie koła są beznapędowe
        info.fx_fr = 0.0; // przednie koła są beznapędowe
        info.fx_rl = x.fx_rl;
        info.fx_rr = x.fx_rr;

        info.Power_total = (x.torque) * (x.omega_rr + x.omega_rl)/2 / 1000 ; // kW

       

        info.torque = x.torque;
        info.torque_left = x.torque_left;
        info.torque_right = x.torque_right;

        info.omega_rl = x.omega_rl;
        info.omega_rr = x.omega_rr;

        info.delta_left = x.delta_left * 180 / M_PI;
        info.delta_rigth = x.delta_right * 180 / M_PI;
        info.rack_angle = x.rack_angle * 180 / M_PI;

        info.ax = x.ax_prev/9.81; // because in g units i opozninoe o o jeden krok ale to nie szkoda
        info.ay = x. ay_prev/9.81; // because in g units i opozninoe o o jeden krok ale to nie szkoda

        info.yaw_rate = x.yaw_rate;
        info.vx = x.vx;
        info.vy = x.vy

        info.time = step_number * P.get("simulation_time_step");

        info.rack_angle_request = u.rack_angle_request * 180 / M_PI;
        info.torque_request = u.torque_request;
        info.x = x.x;
        info.y = x.y;
        info.yaw = x.yaw;
        info.total_drag = P.get("Cd") * x.vx * x.vx + P.get("Cr") * ( P.get("m") * P.get("g") + P.get("Cl1") * x.vx * x.vx + P.get("Cl2") * x.vx * x.vx ) ;
        info.total_downforce = P.get("Cl1") * x.vx * x.vx + P.get("Cl2") * x.vx * x.vx;

        return info;
}

    void euler_sim_timestep(State& x, const Input& u, const ParamBank& P)){
        double dt = P.get("simulation_time_step");
        State dx = model_derative(P,x,u);
        x += dx * dt;
        unwrap_angle(x.yaw);
    }
    void rk4_sim_timestep(State& x, const Input& u, const ParamBank& P){
        double dt = P.get("simulation_time_step");
        State k1 = model_derative(P,x,u);
        State k2 = model_derative(P,x + 0.5 * dt * k1,u);
        State k3 = model_derative(P,x + 0.5 * dt * k2,u);
        State k4 = model_derative(P,x + dt * k3,u);
        x += (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
        unwrap_angle(x.yaw);
    }

}