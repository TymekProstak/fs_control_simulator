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

        //power limitation for total torque assuming rear wheel drive and open differential and stupid control
        if( x.torque *  (x.omega_rr + x.omega_rl)/2 > P.get("P_max_drive") ){
            temp.torque = 0.0;
            temp.torque_right = 0.0;
            temp.torque_left = 0.0;
        }
        if( x.torque * (x.omega_rr + x.omega_rl)/2 < P.get("P_min_recup")/2 ){
            temp.torque = 0.0;
            temp.torque_right = 0.0;
            temp.torque_left = 0.0;
        }

    
        return temp;
    }


    
}