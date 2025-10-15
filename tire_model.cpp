
#include "tire_model.hpp"

namespace lem_dynamics_sim_{



State derative_tire_model( const  ParamBank& P, const State& x, const Input& u){

    State temp;
    temp.setZero();

    //  zczytanie przydatnych parametrów dynamiki nadwozia 

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

    // parametry opon MF 5.2 uproszczony
    

    double pCx1 = P.get("pCx1");
    double pEx1 = P.get("pEx1");
    double pDx1 = P.get("pDx1");
    double pDx2 = P.get("pDx2");
    double pKx1 = P.get("pKx1");
    double pKx3 = P.get("pKx3");
    double lambda_x = P.get("lambda_x");

    double pCy1 = P.get("pCy1");
    double pEy1 = P.get("pEy1");
    double pDy1 = P.get("pDy1");
    double pDy2 = P.get("pDy2");
    double pKy1 = P.get("pKy1");
    double pKy2 = P.get("pKy2");
    double lambda_y = P.get("lambda_y");

    double N0 = P.get("N0");


    // sztywności zawieszenia

    double Kf = P.get("Kf");
    double Kr = P.get("Kr");
    double K_total =  Kf + Kr;
    double mf = m * a/w;
    double mr = m * b/w;
    double h_prim_f = h - h_roll_f;
    double h_prim_r = h - h_roll_r;

    // korekta o epsilon by nie było singularities przy vx = 0
 
    double epsilon = P.get("epsilon");
    double cx = P.get("cx"); 
    double dx = P.get("dx");
    double cy = P.get("cy");
    double dy = P.get("dy");

    // pozycje kół

    double r_rear = P.get("r_rear");
    double r_front = P.get("r_front");
    double angle_construction_front = P.get("angle_construction_front");
    double angle_construction_rear = P.get("angle_construction_rear");
    double R = P.get("R");

     // kinematyka 


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

    

   // transfer masy :  https://kktse.github.io/jekyll/update/2021/05/12/simplied-lateral-load-transfer-analysis.html

   double N_fl = 0.5 * mf * g - 0.5 * m * x.prev_ax * h/w  - x.prev_ay/t_front * ( mf * h_roll_f + Kf/K_total *(mf * h_prim_f + mr * h_prim_r)) + 1/2 * P.get("Cl1") * x.vx * x.vx ;
   double N_fr = 0.5 * mf * g  - 0.5 * m * x.prev_ax * h/w  + x.prev_ay /t_front * ( mf * h_roll_f + Kf/K_total *(mf * h_prim_f + mr * h_prim_r)) + 1/2 * P.get("Cl1") * x.vx * x.vx ;
   double N_rl = 0.5 * mr * g  + 0.5 * m * x.prev_ax * h/w  - x.prev_ay/t_rear * ( mr * h_roll_r + Kr/K_total *(mf * h_prim_f + mr * h_prim_r)) + 1/2 * P.get("Cl2") * x.vx * x.vx ;
   double N_rr = 0.5 * mr *g   + 0.5 * m * x.prev_ax * h/w  + x.prev_ay/t_rear * ( mr * h_roll_r + Kr/K_total *(mf * h_prim_f + mr * h_prim_r)) + 1/2 * P.get("Cl2") * x.vx * x.vx ;


    // kinematyka statycznego slipu

    double slip_angle_fr = x.delta_right - std::atan2(vy_fr,vx_fr_denom) ;
    double slip_angle_fl = x.delta_left  - std::atan2(vy_fl,vx_fl_denom) ;
    double slip_angle_rr =  - std::atan2(vy_rr,vx_rl_denom);
    double slip_angle_rl =  - std::atan2(vy_rl,vx_rl_denom);

    slip_angle_fr = std::clamp(slip_angle_fl,-M_PI/2 + 0.01,M_PI/2 - 0.01); // so dosen't go to infinity
    slip_angle_fl = std::clamp(slip_angle_fr,-M_PI/2 + 0.01 , M_PI/2 - 0.01); // so dosen't go to infinity
    slip_angle_rr = std::clamp(slip_angle_rl,-M_PI/2 + 0.01 , M_PI/2 - 0.01); // so dosen't go to infinity
    slip_angle_rl = std::clamp(slip_angle_rr,-M_PI/2 + 0.01 , M_PI/2 - 0.01); // so dosen't go to infinity


    double slip_ratio_fr = 0.0; // przednie koła są beznapędwoe
    double slip_ratio_fl = 0.0; // przednie koła są beznapędowe

    
    double slip_ratio_rr = (x.omega_rr * R - vx_rr )/vx_rr_denom;
    double slip_ratio_rl = (x.omega_rl * R - vx_rl) / vx_rl_denom;


    // clamping slips -> pytanie czy jak zrobie do -1,1 to nie usunę jakiś excessiv slipa? chuj wie
    // clamping slips -> pytanie czy jak zrobię -1,1 to nie usnunę jakiś excsessiv slip ? chuj wie
    slip_ratio_rr =  std::clamp(slip_ratio_rr,-0.99, 0.99); 
    slip_ratio_fl =  std::clamp(slip_ratio_fl,-0.99, 0.99);

    // liczenie argumentów MF 5.2 uproszczonego statycznego - https://www.researchgate.net/publication/332637470_A_free-trajectory_quasi-steady-state_optimal-control_method_for_minimum_lap-time_of_race_vehicles

    double dfz_fl = (N_fl - N0)/N0 ;
    double dfz_fr = (N_fr - N0)/N0 ;
    double dfz_rr = (N_rr - N0)/N0 ;
    double dfz_rl =  (N_rl - N0)/N0 ;

    double slip_x_fl = 0.0;
    double slip_x_fr = 0.0;
    double slip_x_rr = slip_ratio_rr/ (1 + slip_ratio_rr ) + 1e-6; // so it cancles with denom in slip chuj wie
    double slip_x_rl = slip_ratio_rl/ (1 + slip_ratio_rl ) + 1e-6; // so it cancles with denom in slip chuj wie


    double slip_y_fl = std::tan(slip_angle_fl) / (1 + slip_ratio_fl ) + 1e-6; // so it cancles with denom in slip chuj wie
    double slip_y_fr = std::tan(slip_angle_fr) / (1 + slip_ratio_fr ) + 1e-6; // so it cancles with denom in slip chuj wie
    double slip_y_rr = std::tan(slip_angle_rr) / (1 + slip_ratio_rr ) ; 
    double slip_y_rl = std::tan(slip_angle_rl) / (1 + slip_ratio_rl ) ;

    double slip_fl = std::sqrt(slip_x_fl*slip_x_fl + slip_y_fl*slip_y_fl ) + 1e-6 ; // to avoid division by zero chuj wie
    double slip_fr = std::sqrt(slip_x_fr*slip_x_fr + slip_y_fr*slip_y_fr ) + 1e-6; // to avoid division by zero chuj wie
    double slip_rr = std::sqrt(slip_x_rr*slip_x_rr + slip_y_rr*slip_y_rr)  + 1e-6; // to avoid division by zero chuj wie
    double slip_rl = std::sqrt(slip_x_rl*slip_x_rl + slip_y_rl*slip_y_rl ) + 1e-6; // to avoid division by zero chuj wie

    // liczenie sił  w pacejka frame statycznego

        // makroparametry z mikroparametorwow
        double Ex_fl = pEx1;
        double Ex_fr = pEx1;
        double Ex_rl = pEx1;
        double Ex_rr = pEx1;
        double Dx_fl = lambda_x* (pDx1 + pDx2 * dfz_fl);
        double Dx_fr = lambda_x * (pDx1 + pDx2 * dfz_fr);
        double Dx_rl = lambda_x * (pDx1 + pDx2 * dfz_rl);
        double Dx_rr = lambda_x * (pDx1 + pDx2 * dfz_rr);
        double Cx_fl = pCx1;
        double Cx_fr = pCx1;
        double Cx_rl = pCx1;
        double Cx_rr = pCx1;
        double Kx_fl = N_fl * pKx1  * std::exp(pKx3 * dfz_fl);
        double Kx_fr = N_fr * pKx1  * std::exp(pKx3 * dfz_fr);
        double Kx_rl = N_rl * pKx1  * std::exp(pKx3 * dfz_rl) ;
        double Kx_rr = N_rr * pKx1  * std::exp(pKx3 * dfz_rr) ;
        double Bx_fl = Kx_fl/Cx_fl/Dx_fl/N_fl ;
        double Bx_fr = Kx_fr/Cx_fr/Dx_fr/N_fr  ;
        double Bx_rl = Kx_rl/Cx_rl/Dx_rl/N_rl  ;
        double Bx_rr = Kx_rr/Cx_rr/Dx_rr/N_rr  ;

        double Ey_fl = pEy1;
        double Ey_fr = pEy1;
        double Ey_rl = pEy1;
        double Ey_rr = pEy1;
        double Dy_fl = lambda_y * (pDy1 + pDy2 * dfz_fl);
        double Dy_fr = lambda_y * (pDy1 + pDy2 * dfz_fr);
        double Dy_rl = lambda_y * (pDy1 + pDy2 * dfz_rl);
        double Dy_rr = lambda_y * (pDy1 + pDy2 * dfz_rr);
        double Cy_fl = pCy1;
        double Cy_fr = pCy1;
        double Cy_rl = pCy1;
        double Cy_rr = pCy1;
        double Ky_fl = N0 * pKy1  * std::sin( 2 * std::atan2( N_fl ,(pKy2 * N0)));
        double Ky_fr = N0 * pKy1  * std::sin(2 * std::atan2( N_fr ,(pKy2 * N0)));
        double Ky_rl = N0 * pKy1  * std::sin(2 * std::atan2( N_rl ,(pKy2 * N0)));
        double Ky_rr = N0 * pKy1  * std::sin(2 * std::atan2( N_rr,(pKy2 * N0)));
        double By_fl = Ky_fl/Cy_fl/Dy_fl/N_fl ;
        double By_fr = Ky_fr/Cy_fr/Dy_fr/N_fr  ;
        double By_rl = Ky_rl/Cy_rl/Dy_rl/N_rl  ;
        double By_rr = Ky_rr/Cy_rr/Dy_rr/N_rr  ;


    // wstawienie do funkcji pacejka

    /// front

    double Fx_fl = 0.0 ;// przednie koła są beznapędowe

    double Fy_fl_static = N_fl*slip_y_fl/slip_fl * Dy_fl*std::sin(Cy_fl * std::atan(By_fl * slip_fl - Ey_fl*(By_fl * slip_fl - std::atan(By_fl * slip_fl ) )));
    double d_Fy_fl_static_d_slip_angle =  dFy_dslipY(   slip_x_fl,  slip_y_fl,  N_fl,  Dy_fl,  Cy_fl,  By_fl,  Ey_fl);
    

    double Fx_fr = 0.0 ; // przednie koła są beznapędowe
    
    double Fy_fr_static =  N_fr*slip_y_fl/slip_fr* Dy_fr*std::sin(Cy_fr * std::atan(By_fr * slip_fr - Ey_fr*(By_fr * slip_fr - std::atan(By_fr * slip_fr ) ))); 
    double d_Fy_fr_static_d_slip_angle = dFy_dslipY(   slip_x_fr,  slip_y_fr,  N_fr,  Dy_fr,  Cy_fr,  By_fr,  Ey_fr);

    /// rear

    double Fx_rl_static = N_rl*slip_x_rl/slip_rl * Dx_rl*std::sin(Cx_rl * std::atan(Bx_rl * slip_rl - Ex_rl*(Bx_rl * slip_rl - std::atan(Bx_rl * slip_rl ) ))); 
    double d_Fx_rl_static_d_slip_ratio = dFx_dslipX(   slip_x_rl,  slip_y_rl,  N_rl,  Dx_rl,  Cx_rl,  Bx_rl,  Ex_rl);

    double Fy_rl_static = N_rl*slip_y_rl/slip_rl* Dy_rl*std::sin(Cy_rl * std::atan(By_rl * slip_rl - Ey_rl*(By_rl * slip_rl - std::atan(By_rl * slip_rl ) )));
    double d_Fy_rl_static_d_slip_angle =  dFy_dslipY(   slip_x_rl,  slip_y_rl,  N_rl,  Dy_rl,  Cy_rl,  By_rl,  Ey_rl);

    double Fx_rr_static = N_rr*slip_x_rr/slip_rr* Dx_rr*std::sin(Cx_rr * std::atan(Bx_rr * slip_rr - Ex_rr*(Bx_rr * slip_rr - std::atan(Bx_rr * slip_rr ) ))); 
    double d_Fx_rr_static_d_slip_ratio = dFx_dslipX(   slip_x_rr,  slip_y_rr,  N_rr,  Dx_rr,  Cx_rr,  Bx_rr,  Ex_rr);

    double Fy_rr_static = N_rr*slip_y_rr/slip_rr* Dy_rr*std::sin(Cy_rr * std::atan(By_rr * slip_rr - Ey_rr*(By_rr * slip_rr - std::atan(By_rr * slip_rr ) ))); 
    double d_Fy_rr_static_d_slip_angle = dFy_dslipY(   slip_x_rr,  slip_y_rr,  N_rr,  Dy_rr,  Cy_rr,  By_rr,  Ey_rr);
    

    // liczenie dynamicznego relax length -  https://www.researchgate.net/publication/251107159_First_Order_Tire_Dynamics

    double dynamic_relax_length_slip_angle_fl = vx_fl_denom * dy/cy + 1/cy * d_Fy_fl_static_d_slip_angle;
    double dynamic_relax_length_slip_angle_fr = vx_fr_denom * dy/cy + 1/cy * d_Fy_fr_static_d_slip_angle;
    double dynamic_relax_length_slip_angle_rl = vx_rl_denom * dy/cy + 1/cy * d_Fy_rl_static_d_slip_angle;
    double dynamic_relax_length_slip_angle_rr = vx_rr_denom * dy/cy + 1/cy * d_Fy_rr_static_d_slip_angle;

    double dynamic_relax_length_slip_ratio_rl = vx_rl_denom * dx/cx + 1/cx * d_Fx_rl_static_d_slip_ratio;
    double dynamic_relax_length_slip_ratio_rr =  vx_rr_denom * dx/cx + 1/cx * d_Fx_rr_static_d_slip_ratio;

   // liczenie pochodnych sił z  modelu relaksacyjnym opóźnieniem - https://www.researchgate.net/publication/251107159_First_Order_Tire_Dynamics

   temp.fy_fl = (Fy_fl_static - x.fy_fl)/dynamic_relax_length_slip_angle_fl * vx_fl_denom;
   temp.fy_fr = (Fy_fr_static - x.fy_fr)/dynamic_relax_length_slip_angle_fr * vx_fr_denom;
   temp.fy_rl = (Fy_rl_static - x.fy_rl)/dynamic_relax_length_slip_angle_rl * vx_rl_denom;
   temp.fy_rr = (Fy_rr_static - x.fy_rr)/dynamic_relax_length_slip_angle_rr * vx_rr_denom;

   temp.fx_rl = (Fx_rl_static - x.fx_rl)/dynamic_relax_length_slip_ratio_rl * vx_rl_denom;
   temp.fx_rr = (Fx_rr_static - x.fx_rr)/dynamic_relax_length_slip_ratio_rr * vx_rr_denom;

    return temp;


}
}