#pragma once
#include "ParamBank.hpp"
#include "uttilities.hpp"


// zakładamy że układ 1 rzedu , a dyferencjał otwarty -> obecny stan gdy lem rozjebany
namespace lem_dynamics_sim_{


    inline State derative_drivetrain(const ParamBank& P, const  State& x, const Input& u) {
        State temp;
        temp.setZero();
        // first order system
        temp.torque = (u.torque_request - x.torque) / P.get("drivetrain_timescale");
        temp.torque_right = (u.torque_request/2- x.torque_right) / P.get("drivetrain_timescale"); ;
        temp.torque_left = (u.torque_request/2 - x.torque_left) / P.get("drivetrain_timescale");

    
    
        return temp;
    }


    
}