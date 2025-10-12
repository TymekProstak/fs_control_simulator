#include "uttilities.hpp"
#include "ParamBank.hpp"

namespace lem_dynamics_sim_{

    struct delta_angels{
        double delta_left;
        double delta_right;
    };


    //// tutaj trzeba zaimplementować funckję akermana itd

    inline delta_angles anit_akerman(double rack_angle , ParamBank& P){

        // póki co mamy double track bo trzeba zerknąc do CADA
        return rack_angle;

    }

    // to samo co wyżej tylko zrożniczkowane  wzg rack angle

    inline delta_angles derative_with_respect_to_rack_angle_anti_akerman(double rack_angle , ParamBank& P){

        // póki co mamy double track bo trzeba zerknąc do CAD
        
        return {1,1};
    }


    State derative_steering(const ParamBank& P, const State& x, const Input& u) ;



}