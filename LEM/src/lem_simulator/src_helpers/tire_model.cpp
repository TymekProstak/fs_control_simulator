#include "tire_model.hpp"
#include <algorithm>
#include <cmath>

namespace lem_dynamics_sim_{

State derative_tire_model(const ParamBank& P, const State& x, const Input& u)
{
    State temp; temp.setZero();

    // ============================================================
    // 1) Parametry pojazdu
    // ============================================================
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

    const double Kf = P.get("K1");
    const double Kr = P.get("K2");
    const double K_total = Kf + Kr;

    const double mf = m * a / w;
    const double mr = m * b / w;
    const double h_prim_f = h - h_roll_f;
    const double h_prim_r = h - h_roll_r;

    // ============================================================
    // 2) Parametry “numerical safety” + relaksacje
    // ============================================================
    const double epsilon = P.get("epsilon");
    const double cx = P.get("cx");
    const double dx = P.get("dx");
    const double cy = P.get("cy");
    const double dy = P.get("dy");

    const double dt = P.get("simulation_time_step");

    // ============================================================
    // 3) Geometria kół + promień
    // ============================================================
    const double r_rear  = P.get("r_rear");
    const double r_front = P.get("r_front");
    const double ang_f = P.get("angle_construction_front");
    const double ang_r = P.get("angle_construction_rear");
    const double R = P.get("R");

    // ============================================================
    // 4) MF 6.1 - parametry (no camber)
    //    Ja zakładam, że w ParamBank masz te klucze.
    // ============================================================

    // -------- LONGITUDINAL MF6.1 --------
    const double pCx1 = P.get("pCx1");

    const double pDx1 = P.get("pDx1");
    const double pDx2 = P.get("pDx2");
    // const double pDx3 = P.get("pDx3"); // camber term, ignoruję

    const double pEx1 = P.get("pEx1");
    const double pEx2 = P.get("pEx2");
    const double pEx3 = P.get("pEx3");
    const double pEx4 = P.get("pEx4");

    const double pKx1 = P.get("pKx1");
    const double pKx2 = P.get("pKx2");
    const double pKx3 = P.get("pKx3");

    const double pHx1 = P.get("pHx1");
    const double pHx2 = P.get("pHx2");

    const double pVx1 = P.get("pVx1");
    const double pVx2 = P.get("pVx2");

    const double lambda_x = P.get("lambda_x");

    // -------- LATERAL MF6.1 --------
    const double pCy1 = P.get("pCy1");

    const double pDy1 = P.get("pDy1");
    const double pDy2 = P.get("pDy2");
    // const double pDy3 = P.get("pDy3"); // camber^2 term, ignoruję

    const double pEy1 = P.get("pEy1");
    const double pEy2 = P.get("pEy2");
    const double pEy3 = P.get("pEy3");
    // const double pEy4 = P.get("pEy4"); // camber term, ignoruję
    // const double pEy5 = P.get("pEy5"); // camber term, ignoruję

    const double pKy1 = P.get("pKy1");
    const double pKy2 = P.get("pKy2");
    const double pKy4 = P.get("pKy4");

    const double pHy1 = P.get("pHy1");
    const double pHy2 = P.get("pHy2");

    const double pVy1 = P.get("pVy1");
    const double pVy2 = P.get("pVy2");
    // const double pVy3 = P.get("pVy3"); // camber shift, ignoruję
    // const double pVy4 = P.get("pVy4"); // camber shift, ignoruję

    const double lambda_y = P.get("lambda_y");

    const double N0 = P.get("N0");

    // ============================================================
    // 5) Kinematyka kół (lokalne prędkości)
    // ============================================================
    const double vx_rr = x.vx + x.yaw_rate * r_rear  * std::sin(ang_r);
    const double vy_rr = x.vy - x.yaw_rate * r_rear  * std::cos(ang_r);

    const double vx_rl = x.vx - x.yaw_rate * r_rear  * std::sin(ang_r);
    const double vy_rl = x.vy - x.yaw_rate * r_rear  * std::cos(ang_r);

    const double vx_fr = x.vx + x.yaw_rate * r_front * std::sin(ang_f);
    const double vy_fr = x.vy + x.yaw_rate * r_front * std::cos(ang_f);

    const double vx_fl = x.vx - x.yaw_rate * r_front * std::sin(ang_f);
    const double vy_fl = x.vy + x.yaw_rate * r_front * std::cos(ang_f);

    // ---- Ja robię bezpieczne vx do atan2: zachowuję znak, ale daję podłogę.
    auto safe_vx = [&](double vx){
        return std::copysign(std::max(std::abs(vx), epsilon), vx);
    };

    // ---- Ja robię “symetryczny” mianownik do slip ratio (żeby nie wybuchło przy 0)
    auto vx_denom_kappa = [&](double vx, double omegaR){
        return std::max({std::abs(vx), std::abs(omegaR), epsilon});
    };

    const double vx_rr_den_kappa = vx_denom_kappa(vx_rr, x.omega_rr * R);
    const double vx_rl_den_kappa = vx_denom_kappa(vx_rl, x.omega_rl * R);

    // ============================================================
    // 6) Obciążenia normalne (jak u Ciebie)
    // ============================================================
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

    // ============================================================
    // 7) Kąty poślizgu (pure lateral input)
    // ============================================================
    double alpha_fr = x.delta_right - std::atan2(vy_fr, safe_vx(vx_fr));
    double alpha_fl = x.delta_left  - std::atan2(vy_fl, safe_vx(vx_fl));
    double alpha_rr = -std::atan2(vy_rr, safe_vx(vx_rr));
    double alpha_rl = -std::atan2(vy_rl, safe_vx(vx_rl));

    alpha_fr = std::clamp(alpha_fr, -M_PI/2 + 0.3, M_PI/2 - 0.3);
    alpha_fl = std::clamp(alpha_fl, -M_PI/2 + 0.3, M_PI/2 - 0.3);
    alpha_rr = std::clamp(alpha_rr, -M_PI/2 + 0.3, M_PI/2 - 0.3);
    alpha_rl = std::clamp(alpha_rl, -M_PI/2 + 0.3, M_PI/2 - 0.3);

    // ============================================================
    // 8) Slip ratio (pure longitudinal input)
    // ============================================================
    // Ja liczę kappa jako klasyczne slip_ratio (symetrycznie),
    // a potem clampa, bo MF nie lubi ekstremów.
    const double kappa_rr = std::clamp((x.omega_rr * R - vx_rr) / vx_rr_den_kappa, -0.99, 0.99);
    const double kappa_rl = std::clamp((x.omega_rl * R - vx_rl) / vx_rl_den_kappa, -0.99, 0.99);

    // Front (u Ciebie brak napędu) → trzymam kappa = 0
    const double kappa_fl = 0.0;
    const double kappa_fr = 0.0;

    // ============================================================
    // 9) dfz
    // ============================================================
    const double dfz_fl = (N_fl - N0) / N0;
    const double dfz_fr = (N_fr - N0) / N0;
    const double dfz_rl = (N_rl - N0) / N0;
    const double dfz_rr = (N_rr - N0) / N0;

    // ============================================================
    // 10) Helper: MF6.1 pure + pochodna dF/dx
    // ============================================================
    auto mf61_pure = [&](double x_in, double C, double D, double B, double E, double Sv,
                         double& F, double& dFdx)
    {
        // Ja liczę:
        // F = D * sin(C*atan(phi)) + Sv
        // phi = Bx - E*(Bx - atan(Bx)), Bx = B*x
        const double Bx = B * x_in;
        const double atanBx = std::atan(Bx);
        const double phi = Bx - E * (Bx - atanBx);
        const double atanPhi = std::atan(phi);

        F = D * std::sin(C * atanPhi) ;//+ Sv;

        // dF/dx
        // dF/dx = D*cos(C*atanPhi)*C*(1/(1+phi^2))*dphi/dx
        // dphi/dx = B - E*(B - B/(1+Bx^2))
        const double d_atanPhi_dphi = 1.0 / (1.0 + phi * phi);
        const double d_atanBx_dBx   = 1.0 / (1.0 + Bx * Bx);

        const double dphi_dx = B - E * (B - B * d_atanBx_dBx);

        dFdx = D * std::cos(C * atanPhi) * C * d_atanPhi_dphi * dphi_dx;
    };

    // ============================================================
    // 11) Helper: Elipsa tarcia + pochodne (tylko w swoich osiach)
    // ============================================================
    auto ellipse_couple = [&](double Fx0, double Fy0,
                              double FxMax, double FyMax,
                              double dFx0_dkappa, double dFy0_dalpha,
                              double& Fx, double& Fy,
                              double& dFx_dkappa, double& dFy_dalpha)
    {
        const double eps = 1e-9;
        FxMax = std::max(std::abs(FxMax), eps);
        FyMax = std::max(std::abs(FyMax), eps);

        const double ax = Fx0 / FxMax;
        const double ay = Fy0 / FyMax;

        const double s = std::sqrt(ax * ax + ay * ay);

        if (s <= 1.0 || !std::isfinite(s))
        {
            Fx = Fx0;
            Fy = Fy0;
            dFx_dkappa = dFx0_dkappa;
            dFy_dalpha = dFy0_dalpha;
            return;
        }

        const double invs = 1.0 / s;

        Fx = Fx0 * invs;
        Fy = Fy0 * invs;

        // Ja liczę pochodne dokładnie dla naszego przypadku:
        //
        // s = sqrt( (Fx0/FxMax)^2 + (Fy0/FyMax)^2 )
        //
        // d/dkappa: tylko Fx0 zależy od kappa (Fy0 ~ const)
        // ds/dkappa = (1/s)*(Fx0/FxMax^2)*dFx0/dkappa
        //
        // d(Fx0/s)/dkappa = (dFx0*s - Fx0*ds)/s^2
        const double ds_dkappa =
            (1.0 / s) * (Fx0 / (FxMax * FxMax)) * dFx0_dkappa;

        const double ds_dalpha =
            (1.0 / s) * (Fy0 / (FyMax * FyMax)) * dFy0_dalpha;

        dFx_dkappa = (dFx0_dkappa * s - Fx0 * ds_dkappa) / (s * s);
        dFy_dalpha = (dFy0_dalpha * s - Fy0 * ds_dalpha) / (s * s);
    };

   

   

    // ============================================================
    // 14) Liczę PURE Fx0(kappa) i Fy0(alpha) + coupling elipsą
    // ============================================================

    auto compute_wheel_forces = [&](double Fz, double dfz,
        double kappa, double alpha,
        bool allow_longitudinal,
        double& Fx, double& Fy,
        double& dFx_dkappa, double& dFy_dalpha,
        double& mu_x, double& mu_y)
        {
        // ---------- LONG pure ----------
        double Cx, mux, Dx, Kxk, Bx, Ex, Shx, Svx;

        // parametry long (MF6.1, camber=0)
        {
            Cx = pCx1;// * lambda_x;

            mux = lambda_x * (pDx1 + pDx2 * dfz);
            mux = std::max(mux, 1e-6);
            Dx  = mux * Fz;

            Kxk = Fz * (pKx1 + pKx2 * dfz) * std::exp(pKx3 * dfz);

            Shx = pHx1 + pHx2 * dfz;
            Svx = Fz * (pVx1 + pVx2 * dfz);

            //Shx = 0.0; // jeśli chcesz bez shiftów, to tu daj 0
            Svx = 0.0; // jeśli chcesz bez shiftów, to tu

            const double denom = std::max(Cx * Dx, 1e-9);
            Bx = Kxk / denom;

            const double E0 = (pEx1 + pEx2 * dfz + pEx3 * dfz * dfz);
            const double kx_sign = kappa + Shx;
            auto smooth_sign = [&](double x){
                const double s = 50.0; // ostrość
                return std::tanh(s * x);
            };
            Ex = E0 * (1.0 - pEx4 * smooth_sign(kappa + Shx));
        }

        double Fx0 = 0.0;
        double dFx0_dk = 0.0;

        if (allow_longitudinal)
        {
            const double kx = kappa + Shx;  // PURE input + shift
            mf61_pure(kx, Cx, Dx, Bx, Ex, Svx, Fx0, dFx0_dk);
            
            double Fx_raw = Fx0;
             double Fx_bias = 0.0, dtmp = 0.0;
            {
                const double kx0 = 0.0 + Shx;
                mf61_pure(kx0, Cx, Dx, Bx, Ex, Svx, Fx_bias, dtmp);
            }

            // Final:
            Fx0 = Fx_raw - Fx_bias;
        }



        // ---------- LAT pure ----------
        double Cy, muy, Dy, Kya, By, Ey, Shy, Svy;

        {
        Cy = pCy1;

            muy = lambda_y * (pDy1 + pDy2 * dfz);
            muy = std::max(muy, 1e-6);
            Dy  = muy * Fz;

            Kya = pKy1 * N0 * std::sin(pKy4 * std::atan(Fz / (pKy2 * N0)));

            const double denom = std::max(Cy * Dy, 1e-9);
            By = Kya / denom;

            Shy = pHy1 + pHy2 * dfz;
            Svy = Fz * (pVy1 + pVy2 * dfz);
            //Shy = 0.0; // jeśli chcesz bez shiftów, to tu daj 0
            Svy = 0.0; // jeśli chcesz bez shiftów, to tu
            
            auto smooth_sign = [&](double x){
                const double s = 50.0; // ostrość
                return std::tanh(s * x);
            };
            Ey = (pEy1 + pEy2 * dfz) *(1 - pEy3*smooth_sign(alpha + Shy));
        }

        const double ay = alpha + Shy;  // PURE input + shift
        double Fy0 = 0.0;
        double dFy0_da = 0.0;
        mf61_pure(ay, Cy, Dy, By, Ey, Svy, Fy0, dFy0_da);

        double Fy_raw = Fy0;

        double Fy_bias = 0.0, dtmp2 = 0.0;
        {
            const double ay0 = 0.0 + Shy;
            mf61_pure(ay0, Cy, Dy, By, Ey, Svy, Fy_bias, dtmp2);
        }

        Fy0 = Fy_raw - Fy_bias;

        // ---------- COUPLING ELLIPSE (ONLY forces!) ----------
        const double FxMax = mux * Fz;
        const double FyMax = muy * Fz;

        const double eps = 1e-9;
        const double FxMaxSafe = std::max(std::abs(FxMax), eps);
        const double FyMaxSafe = std::max(std::abs(FyMax), eps);

        const double axu = Fx0 / FxMaxSafe;
        const double ayu = Fy0 / FyMaxSafe;

        const double s = std::sqrt(axu * axu + ayu * ayu);

        if (s <= 1.0 || !std::isfinite(s))
        {
            Fx = Fx0;
            Fy = Fy0;
        }
        else
        {
            const double invs = 1.0 / s;
            Fx = Fx0 * invs;
            Fy = Fy0 * invs;
        }

        // ---------- DERIVATIVES FOR RELAXATION = PURE ONLY ----------
        dFx_dkappa = dFx0_dk;
        dFy_dalpha = dFy0_da;

        mu_x = mux;
        mu_y = muy;
        };


    // ============================================================
    // 15) Siły na kołach + pochodne
    // ============================================================

    // FRONT: brak napędu (Fx=0), ale Fy liczę normalnie + elipsa (tu elipsa nic nie popsuje)
    double Fx_fl, Fy_fl, dFx_fl_dk, dFy_fl_da, mux_fl, muy_fl;
    double Fx_fr, Fy_fr, dFx_fr_dk, dFy_fr_da, mux_fr, muy_fr;

    compute_wheel_forces(N_fl, dfz_fl, kappa_fl, alpha_fl, false,
                         Fx_fl, Fy_fl, dFx_fl_dk, dFy_fl_da, mux_fl, muy_fl);

    compute_wheel_forces(N_fr, dfz_fr, kappa_fr, alpha_fr, false,
                         Fx_fr, Fy_fr, dFx_fr_dk, dFy_fr_da, mux_fr, muy_fr);

    // REAR: napęd (Fx aktywne)
    double Fx_rl, Fy_rl, dFx_rl_dk, dFy_rl_da, mux_rl, muy_rl;
    double Fx_rr, Fy_rr, dFx_rr_dk, dFy_rr_da, mux_rr, muy_rr;

    compute_wheel_forces(N_rl, dfz_rl, kappa_rl, alpha_rl, true,
                         Fx_rl, Fy_rl, dFx_rl_dk, dFy_rl_da, mux_rl, muy_rl);

    compute_wheel_forces(N_rr, dfz_rr, kappa_rr, alpha_rr, true,
                         Fx_rr, Fy_rr, dFx_rr_dk, dFy_rr_da, mux_rr, muy_rr);

    // ============================================================
    // 16) Relaxation lengths (na podstawie Twojej formuły)
    //     Ja używam pochodnych liczonych NA SHIFTACH + z elipsą.
    // ============================================================

    const double Vx_rr = std::max(std::abs(vx_rr), 0.1);
    const double Vx_rl = std::max(std::abs(vx_rl), 0.1);
    const double Vx_fl = std::max(std::abs(vx_fl), 0.1);
    const double Vx_fr = std::max(std::abs(vx_fr), 0.1);

    // dFy/dalpha już mam (pure + coupling)
    
    double dynamic_relax_length_slip_angle_fl = Vx_fl * dy / cy + (1.0 / cy) * std::abs(dFy_fl_da);
    double dynamic_relax_length_slip_angle_fr = Vx_fr * dy / cy + (1.0 / cy) * std::abs(dFy_fr_da);
    double dynamic_relax_length_slip_angle_rl = Vx_rl * dy / cy + (1.0 / cy) * std::abs(dFy_rl_da);
    double dynamic_relax_length_slip_angle_rr = Vx_rr * dy / cy + (1.0 / cy) * std::abs(dFy_rr_da);

    // dFx/dkappa już mam (pure + coupling) – tylko tył
    double dynamic_relax_length_slip_ratio_rl = Vx_rl * dx / cx + (1.0 / cx) * std::abs(dFx_rl_dk);
    double dynamic_relax_length_slip_ratio_rr = Vx_rr * dx / cx + (1.0 / cx) * std::abs(dFx_rr_dk);
    
   /*
   double dynamic_relax_length_slip_angle_fl  = 0.3;
   double dynamic_relax_length_slip_angle_fr  = 0.3;
   double dynamic_relax_length_slip_angle_rl  = 0.3;
   double dynamic_relax_length_slip_angle_rr  = 0.3;
   double dynamic_relax_length_slip_ratio_rl  = 0.3;
   double dynamic_relax_length_slip_ratio_rr  = 0.3;
   */
    // ============================================================
    // 17) ZOH (exact) dla relaksacji
    // ============================================================
    auto zoh_relax_deriv = [&](double F, double Fss, double vx, double L)->double {
        const double V  = std::max(std::abs(vx), epsilon);
        const double Ls = std::max(L, 0.1);
        const double z  = (V / Ls) * dt;
        const double one_minus_a = -std::expm1(-z);  // 1 - e^{-z}
        return (Fss - F) * (one_minus_a / dt);
    };

    // Lateral (wszystkie koła)
    temp.fy_fl = zoh_relax_deriv(x.fy_fl, Fy_fl, vx_fl, dynamic_relax_length_slip_angle_fl);
    temp.fy_fr = zoh_relax_deriv(x.fy_fr, Fy_fr, vx_fr, dynamic_relax_length_slip_angle_fr);
    temp.fy_rl = zoh_relax_deriv(x.fy_rl, Fy_rl, vx_rl, dynamic_relax_length_slip_angle_rl);
    temp.fy_rr = zoh_relax_deriv(x.fy_rr, Fy_rr, vx_rr, dynamic_relax_length_slip_angle_rr);

    // Longitudinal (tylko napęd)
    temp.fx_rl = zoh_relax_deriv(x.fx_rl, Fx_rl, vx_rl, dynamic_relax_length_slip_ratio_rl);
    temp.fx_rr = zoh_relax_deriv(x.fx_rr, Fx_rr, vx_rr, dynamic_relax_length_slip_ratio_rr);

    return temp;
}

} // namespace lem_dynamics_sim_
