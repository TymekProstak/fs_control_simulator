#include "ParamBank.hpp"
#include "uttilities.hpp"
#include "drivetrain.hpp"
#include "tire_model.hpp"
#include "steering_system.hpp"



namespace lem_dynamics_sim_{

    struct Log_Info{


        double kappa_fl; // nondim
        double kappa_fr; // nondim
        double kappa_rl; // nondim
        double kappa_rr; // nondim

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
        

    };

    State model_derative(const ParamBank& P, const  State& x, const Input& u);
    Log_Info log_info(const State& x, const Input& u, const ParamBank& P , int step_number);
    void rk4_sim_timestep(State& x, const Input& u, const ParamBank& P);
    void euler_sim_timestep(State& x, const Input& u, const ParamBank& P);


}