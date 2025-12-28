#include "steering_system.hpp"
 namespace lem_dynamics_sim_
{

    State  derative_steering(const ParamBank& P, const State& x, const Input& u) {
        State temp;
        temp.setZero();

        double omega_n = P.get("natural_frequency_steering_system")  ;
        double omega_n_pow = omega_n*omega_n;

        temp.rack_angle = x.d_rack_angle ;
        temp.d_rack_angle = -2*P.get("steering_system_damping")*omega_n*x.d_rack_angle - omega_n_pow*x.rack_angle + u.rack_angle_request*omega_n_pow ;

        temp.delta_left = (anit_akerman(x.rack_angle , P).delta_right - x.delta_right)/P.get("simulation_time_step");
        temp.delta_right = (anit_akerman(x.rack_angle , P).delta_left - x.delta_left)/P.get("simulation_time_step");

       

        
        return temp;
    }
    
} // namespace namespace lem_dynamics_sim_
