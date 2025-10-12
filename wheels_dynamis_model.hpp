#include "ParamBank.hpp"
#include "uttilities.hpp"

// model convetion (+) omega for clockwise

namespace lem_dynamics_sim_{

    inline State derative_wheels_dynamics_model(ParamBank& P, State& x, const Input& u) {

        State temp;
        temp.setZero();
        temp.omega_rr = (x.torque_left -  x.fx_rr * P.get("R") )/P.get("I_tire");
        temp.omega_rl = (x.torque_rigth  - x.fx_rl * P.get("R") )/P.get("I_tire");;
        return temp;


    }
    
}