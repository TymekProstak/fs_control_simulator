#include "tire_model.hpp"
#include <algorithm>
#include <cmath>

namespace lem_dynamics_sim_{

State derative_tire_model(const ParamBank& P, const State& x, const Input& u)
{
    State temp; temp.setZero();

    // ---- parametry pojazdu / opon
    const double m = P.get("m");
    const double g = P.get("g");
    const double w = P.get("w");
    const double a = P.get("a");
    const double b = P.get("b");
    const double t_front = P.get("t_front");
    const double t_rear  = P.get("t_rear");
    const double h = P.get("h");
    const double h_roll_f = P.get("h1_roll");
    const double h_roll_r = P.get("h2_roll");

    const double pCx1 = P.get("pCx1");
    const double pEx1 = P.get("pEx1");
    const double pDx1 = P.get("pDx1");
    const double pDx2 = P.get("pDx2");
    const double pKx1 = P.get("pKx1");
    const double pKx3 = P.get("pKx3");
    const double lambda_x = P.get("lambda_x");

    const double pCy1 = P.get("pCy1");
    const double pEy1 = P.get("pEy1");
    const double pDy1 = P.get("pDy1");
    const double pDy2 = P.get("pDy2");
    const double pKy1 = P.get("pKy1");
    const double pKy2 = P.get("pKy2");
    const double lambda_y = P.get("lambda_y");

    const double N0 = P.get("N0");

    const double Kf = P.get("K1");
    const double Kr = P.get("K2");
    const double K_total = Kf + Kr;
    const double mf = m * a / w;
    const double mr = m * b / w;
    const double h_prim_f = h - h_roll_f;
    const double h_prim_r = h - h_roll_r;

    const double epsilon = P.get("epsilon");
    const double cx = P.get("cx");
    const double dx = P.get("dx");
    const double cy = P.get("cy");
    const double dy = P.get("dy");

    const double r_rear  = P.get("r_rear");
    const double r_front = P.get("r_front");
    const double ang_f = P.get("angle_construction_front");
    const double ang_r = P.get("angle_construction_rear");
    const double R = P.get("R");

    // ---- kinematyka kół (lokalne prędkości styczne)
    const double vx_rr = x.vx + x.yaw_rate * r_rear  * std::sin(ang_r);
    const double vy_rr = x.vy - x.yaw_rate * r_rear  * std::cos(ang_r);

    const double vx_rl = x.vx - x.yaw_rate * r_rear  * std::sin(ang_r);
    const double vy_rl = x.vy - x.yaw_rate * r_rear  * std::cos(ang_r); 

    const double vx_fr = x.vx + x.yaw_rate * r_front * std::sin(ang_f);
    const double vy_fr = x.vy + x.yaw_rate * r_front * std::cos(ang_f);

    const double vx_fl = x.vx - x.yaw_rate * r_front * std::sin(ang_f);
    const double vy_fl = x.vy + x.yaw_rate * r_front * std::cos(ang_f);

    // ---- bezpieczne vx do atan2 (ze znakiem, z podłogą)
    auto safe_vx = [&](double vx){
        return std::copysign(std::max(std::abs(vx), epsilon), vx);
    };

    // ---- „symetryczny” mianownik tylko do slip ratio (nie do atan2!)
    auto vx_denom_kappa = [&](double vx, double omegaR){
        return std::max({std::abs(vx), std::abs(omegaR), epsilon});
    };

    const double vx_rr_den_kappa = vx_denom_kappa(vx_rr, x.omega_rr * R);
    const double vx_rl_den_kappa = vx_denom_kappa(vx_rl, x.omega_rl * R);
    const double vx_fr_den_kappa = vx_denom_kappa(vx_fr, x.omega_fr * R);
    const double vx_fl_den_kappa = vx_denom_kappa(vx_fl, x.omega_fl * R);

    // ---- quasi-aero + transfer boczny (jak u Ciebie)
    double N_fl = 0.5 * mf * g - 0.5 * m * x.prev_ax * h / w
                - x.prev_ay / t_front * ( mf * h_roll_f + Kf / K_total * (mf * h_prim_f + mr * h_prim_r))
                + 0.5 * P.get("Cl1") * x.vx * x.vx;

    double N_fr = 0.5 * mf * g - 0.5 * m * x.prev_ax * h / w
                + x.prev_ay / t_front * ( mf * h_roll_f + Kf / K_total * (mf * h_prim_f + mr * h_prim_r))
                + 0.5 * P.get("Cl1") * x.vx * x.vx;

    double N_rl = 0.5 * mr * g + 0.5 * m * x.prev_ax * h / w
                - x.prev_ay / t_rear * ( mr * h_roll_r + Kr / K_total * (mf * h_prim_f + mr * h_prim_r))
                + 0.5 * P.get("Cl2") * x.vx * x.vx;

    double N_rr = 0.5 * mr * g + 0.5 * m * x.prev_ax * h / w
                + x.prev_ay / t_rear * ( mr * h_roll_r + Kr / K_total * (mf * h_prim_f + mr * h_prim_r))
                + 0.5 * P.get("Cl2") * x.vx * x.vx;

    const double FZ_MIN = 50.0;
    N_fl = std::max(N_fl, FZ_MIN);
    N_fr = std::max(N_fr, FZ_MIN);
    N_rl = std::max(N_rl, FZ_MIN);
    N_rr = std::max(N_rr, FZ_MIN);

    // ---- kąty poślizgu: UWAGA – atan2(vy, safe_vx(vx))
    double slip_angle_fr = x.delta_right - std::atan2(vy_fr, safe_vx(vx_fr));
    double slip_angle_fl = x.delta_left  - std::atan2(vy_fl, safe_vx(vx_fl));
    double slip_angle_rr = -std::atan2(vy_rr, safe_vx(vx_rr));
    double slip_angle_rl = -std::atan2(vy_rl, safe_vx(vx_rl));

    slip_angle_fr = std::clamp(slip_angle_fr, -M_PI/2 + 0.3, M_PI/2 - 0.3);
    slip_angle_fl = std::clamp(slip_angle_fl, -M_PI/2 + 0.3, M_PI/2 - 0.3);
    slip_angle_rr = std::clamp(slip_angle_rr, -M_PI/2 + 0.3, M_PI/2 - 0.3);
    slip_angle_rl = std::clamp(slip_angle_rl, -M_PI/2 + 0.3, M_PI/2 - 0.3);

    // ---- slip ratio (symetryczna definicja)
    const double slip_ratio_rr = std::clamp((x.omega_rr * R - vx_rr) / vx_rr_den_kappa, -0.99, 0.99);
    const double slip_ratio_rl = std::clamp((x.omega_rl * R - vx_rl) / vx_rl_den_kappa, -0.99, 0.99);
    const double slip_ratio_fr = std::clamp((x.omega_fr * R - vx_fr) / vx_fr_den_kappa, -0.99, 0.99);
    const double slip_ratio_fl = std::clamp((x.omega_fl * R - vx_fl) / vx_fl_den_kappa, -0.99, 0.99);

    // ---- argumenty MF (jak u Ciebie)
    const double dfz_fl = (N_fl - N0) / N0;
    const double dfz_fr = (N_fr - N0) / N0;
    const double dfz_rr = (N_rr - N0) / N0;
    const double dfz_rl = (N_rl - N0) / N0;

    const double slip_x_fl = slip_ratio_fl / (1.0 + slip_ratio_fl);
    const double slip_x_fr = slip_ratio_fr / (1.0 + slip_ratio_fr);
    const double slip_x_rr = slip_ratio_rr / (1.0 + slip_ratio_rr);
    const double slip_x_rl = slip_ratio_rl / (1.0 + slip_ratio_rl);

    const double slip_y_fl = std::tan(slip_angle_fl) / (1.0 + slip_ratio_fl);
    const double slip_y_fr = std::tan(slip_angle_fr) / (1.0 + slip_ratio_fr);
    const double slip_y_rr = std::tan(slip_angle_rr) / (1.0 + slip_ratio_rr);
    const double slip_y_rl = std::tan(slip_angle_rl) / (1.0 + slip_ratio_rl);

    const double slip_fl = std::hypot(slip_x_fl, slip_y_fl) + 1e-6;
    const double slip_fr = std::hypot(slip_x_fr, slip_y_fr) + 1e-6;
    const double slip_rr = std::hypot(slip_x_rr, slip_y_rr) + 1e-6;
    const double slip_rl = std::hypot(slip_x_rl, slip_y_rl) + 1e-6;

    // ---- makroparametry z mikro (jak u Ciebie)
    const double Ex_fl = pEx1, Ex_fr = pEx1, Ex_rl = pEx1, Ex_rr = pEx1;
    const double Dx_fl = lambda_x * (pDx1 + pDx2 * dfz_fl);
    const double Dx_fr = lambda_x * (pDx1 + pDx2 * dfz_fr);
    const double Dx_rl = lambda_x * (pDx1 + pDx2 * dfz_rl);
    const double Dx_rr = lambda_x * (pDx1 + pDx2 * dfz_rr);
    const double Cx_fl = pCx1, Cx_fr = pCx1, Cx_rl = pCx1, Cx_rr = pCx1;

    const double Kx_fl = N_fl * pKx1 * std::exp(pKx3 * dfz_fl);
    const double Kx_fr = N_fr * pKx1 * std::exp(pKx3 * dfz_fr);
    const double Kx_rl = N_rl * pKx1 * std::exp(pKx3 * dfz_rl);
    const double Kx_rr = N_rr * pKx1 * std::exp(pKx3 * dfz_rr);

    const double Bx_fl = Kx_fl / (Cx_fl * Dx_fl * N_fl);
    const double Bx_fr = Kx_fr / (Cx_fr * Dx_fr * N_fr);
    const double Bx_rl = Kx_rl / (Cx_rl * Dx_rl * N_rl);
    const double Bx_rr = Kx_rr / (Cx_rr * Dx_rr * N_rr);

    const double Ey_fl = pEy1, Ey_fr = pEy1, Ey_rl = pEy1, Ey_rr = pEy1;
    const double Dy_fl = lambda_y * (pDy1 + pDy2 * dfz_fl);
    const double Dy_fr = lambda_y * (pDy1 + pDy2 * dfz_fr);
    const double Dy_rl = lambda_y * (pDy1 + pDy2 * dfz_rl);
    const double Dy_rr = lambda_y * (pDy1 + pDy2 * dfz_rr);
    const double Cy_fl = pCy1, Cy_fr = pCy1, Cy_rl = pCy1, Cy_rr = pCy1;

    const double u_fl = N_fl / (pKy2 * N0);
    const double u_fr = N_fr / (pKy2 * N0);
    const double u_rl = N_rl / (pKy2 * N0);
    const double u_rr = N_rr / (pKy2 * N0);

    const double Ky_fl = N0 * pKy1 * (2.0 * u_fl) / (1.0 + u_fl * u_fl);
    const double Ky_fr = N0 * pKy1 * (2.0 * u_fr) / (1.0 + u_fr * u_fr);
    const double Ky_rl = N0 * pKy1 * (2.0 * u_rl) / (1.0 + u_rl * u_rl);
    const double Ky_rr = N0 * pKy1 * (2.0 * u_rr) / (1.0 + u_rr * u_rr);

    const double By_fl = Ky_fl / (Cy_fl * Dy_fl * N_fl);
    const double By_fr = Ky_fr / (Cy_fr * Dy_fr * N_fr);
    const double By_rl = Ky_rl / (Cy_rl * Dy_rl * N_rl);
    const double By_rr = Ky_rr / (Cy_rr * Dy_rr * N_rr);

    // ---- siły statyczne MF
    const double Fx_fl_static = N_fl * (slip_x_fl / slip_fl) * Dx_fl *
        std::sin(Cx_fl * std::atan(Bx_fl * slip_fl - Ex_fl * (Bx_fl * slip_fl - std::atan(Bx_fl * slip_fl))));
    const double d_Fx_fl_dkappa = dFx_dslipX(slip_x_fl, slip_y_fl, N_fl, Dx_fl, Cx_fl, Bx_fl, Ex_fl);

    const double Fy_fl_static = N_fl * (slip_y_fl / slip_fl) * Dy_fl *
        std::sin(Cy_fl * std::atan(By_fl * slip_fl - Ey_fl * (By_fl * slip_fl - std::atan(By_fl * slip_fl))));
    const double d_Fy_fl_dalpha = dFy_dslipY(slip_x_fl, slip_y_fl, N_fl, Dy_fl, Cy_fl, By_fl, Ey_fl);

    const double Fx_fr_static = N_fr * (slip_x_fr / slip_fr) * Dx_fr *
        std::sin(Cx_fr * std::atan(Bx_fr * slip_fr - Ex_fr * (Bx_fr * slip_fr - std::atan(Bx_fr * slip_fr))));
    const double d_Fx_fr_dkappa = dFx_dslipX(slip_x_fr, slip_y_fr, N_fr, Dx_fr, Cx_fr, Bx_fr, Ex_fr);

    const double Fy_fr_static = N_fr * (slip_y_fr / slip_fr) * Dy_fr *
        std::sin(Cy_fr * std::atan(By_fr * slip_fr - Ey_fr * (By_fr * slip_fr - std::atan(By_fr * slip_fr))));
    const double d_Fy_fr_dalpha = dFy_dslipY(slip_x_fr, slip_y_fr, N_fr, Dy_fr, Cy_fr, By_fr, Ey_fr);

    const double Fx_rl_static = N_rl * (slip_x_rl / slip_rl) * Dx_rl *
        std::sin(Cx_rl * std::atan(Bx_rl * slip_rl - Ex_rl * (Bx_rl * slip_rl - std::atan(Bx_rl * slip_rl))));
    const double d_Fx_rl_dkappa = dFx_dslipX(slip_x_rl, slip_y_rl, N_rl, Dx_rl, Cx_rl, Bx_rl, Ex_rl);

    const double Fy_rl_static = N_rl * (slip_y_rl / slip_rl) * Dy_rl *
        std::sin(Cy_rl * std::atan(By_rl * slip_rl - Ey_rl * (By_rl * slip_rl - std::atan(By_rl * slip_rl))));
    const double d_Fy_rl_dalpha = dFy_dslipY(slip_x_rl, slip_y_rl, N_rl, Dy_rl, Cy_rl, By_rl, Ey_rl);

    const double Fx_rr_static = N_rr * (slip_x_rr / slip_rr) * Dx_rr *
        std::sin(Cx_rr * std::atan(Bx_rr * slip_rr - Ex_rr * (Bx_rr * slip_rr - std::atan(Bx_rr * slip_rr))));
    const double d_Fx_rr_dkappa = dFx_dslipX(slip_x_rr, slip_y_rr, N_rr, Dx_rr, Cx_rr, Bx_rr, Ex_rr);

    const double Fy_rr_static = N_rr * (slip_y_rr / slip_rr) * Dy_rr *
        std::sin(Cy_rr * std::atan(By_rr * slip_rr - Ey_rr * (By_rr * slip_rr - std::atan(By_rr * slip_rr))));
    const double d_Fy_rr_dalpha = dFy_dslipY(slip_x_rr, slip_y_rr, N_rr, Dy_rr, Cy_rr, By_rr, Ey_rr);

    // na podstawie https://www.researchgate.net/publication/251107159_First_Order_Tire_Dynamics 

    double dynamic_relax_length_slip_angle_fl = vx_fl * dy/cy + 1.0/cy * d_Fy_fl_dalpha;
    double dynamic_relax_length_slip_angle_fr = vx_fr * dy/cy + 1.0/cy * d_Fy_fr_dalpha;
    double dynamic_relax_length_slip_angle_rl = vx_rl * dy/cy + 1.0/cy * d_Fy_rl_dalpha;
    double dynamic_relax_length_slip_angle_rr = vx_rr * dy/cy + 1.0/cy * d_Fy_rr_dalpha;

    double dynamic_relax_length_slip_ratio_fl = vx_fl * dx/cx + 1/cx * d_Fx_fl_dkappa;
    double dynamic_relax_length_slip_ratio_fr = vx_fr * dx/cx + 1/cx * d_Fx_fr_dkappa;
    double dynamic_relax_length_slip_ratio_rl = vx_rl * dx/cx + 1/cx * d_Fx_rl_dkappa;
    double dynamic_relax_length_slip_ratio_rr =  vx_rr * dx/cx + 1/cx * d_Fx_rr_dkappa;

    // ---- ZOH (exact) dla relaksacji: dF = (Fss - F) * (1 - exp(-k dt)) / dt, k = |vx|/L
    const double dt = P.get("simulation_time_step");
    auto zoh_relax_deriv = [&](double F, double Fss, double vx, double L)->double {
        const double V = std::max(std::abs(vx), epsilon);      // bez gałęzi ωR
        const double Ls = std::max(L, 1e-6);
        const double z  = (V / Ls) * dt;
        const double one_minus_a = -std::expm1(-z);            // 1 - e^{-z}
        return (Fss - F) * (one_minus_a / dt);
    };

    // Lateral
    temp.fy_fl = zoh_relax_deriv(x.fy_fl, Fy_fl_static, vx_fl, dynamic_relax_length_slip_angle_fl);
    temp.fy_fr = zoh_relax_deriv(x.fy_fr, Fy_fr_static, vx_fr, dynamic_relax_length_slip_angle_fr);
    temp.fy_rl = zoh_relax_deriv(x.fy_rl, Fy_rl_static, vx_rl, dynamic_relax_length_slip_angle_rl);
    temp.fy_rr = zoh_relax_deriv(x.fy_rr, Fy_rr_static, vx_rr, dynamic_relax_length_slip_angle_rr);

    // Longitudinal – 4 koła
    temp.fx_fl = zoh_relax_deriv(x.fx_fl, Fx_fl_static, vx_fl, dynamic_relax_length_slip_ratio_fl);
    temp.fx_fr = zoh_relax_deriv(x.fx_fr, Fx_fr_static, vx_fr, dynamic_relax_length_slip_ratio_fr);
    temp.fx_rl = zoh_relax_deriv(x.fx_rl, Fx_rl_static, vx_rl, dynamic_relax_length_slip_ratio_rl);
    temp.fx_rr = zoh_relax_deriv(x.fx_rr, Fx_rr_static, vx_rr, dynamic_relax_length_slip_ratio_rr);

    return temp;
}

}
