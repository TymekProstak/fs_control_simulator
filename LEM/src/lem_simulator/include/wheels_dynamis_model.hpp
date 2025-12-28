#pragma once
#include "ParamBank.hpp"
#include "uttilities.hpp"

// (+) omega clockwise – Twoja konwencja

namespace lem_dynamics_sim_ {

inline State derative_wheels_dynamics_model(const ParamBank& P, const State& x, const Input& u) {
    State temp; temp.setZero();

    const double dt = P.get("simulation_time_step");
    const double I  = P.get("I_tire");
    const double R  = P.get("R");

    // double b0   = 0.0;   // bazowe, małe straty
    // double bLow = 0.6;    // dodatkowe tłumienie przy pełzaniu
    // double Vth  = P.get("epsilon") ;  // m/s – próg "low-speed"
    // auto smoothstep = [](double x){ x = std::clamp(x, 0.0, 1.0); return x*x*(3-2*x); };
    // double sL = smoothstep((Vth - std::abs(x.vx)) / Vth); // 0..1 dla |vx|: 0→Vth
    // double b  = b0 + bLow * sL;
    double b = 0.0;

    auto omega_zoh_next = [&](double omega_k, double T, double Fx)->double {
        const double rhs = (T - Fx * R);       // [N·m]
        if (std::abs(b) > 1e-12) {
            // a = exp(-(b/I)*dt),   gain = (1 - a)/b  (użyj expm1 dla dokładności)
            const double z   = (b / I) * dt;
            const double a   = std::exp(-z);
            const double gain = (-std::expm1(-z)) / b; // = (1 - a)/b
            return a * omega_k + gain * rhs;
        } else {
            // b == 0 → czysty integrator
            return omega_k + (dt / I) * rhs;
        }
    };

    // Uwaga na mapping: LEFT -> RL, RIGHT -> RR
    const double omega_rl_next = omega_zoh_next(x.omega_rl, x.torque_left,  x.fx_rl);
    const double omega_rr_next = omega_zoh_next(x.omega_rr, x.torque_right, x.fx_rr);

    // Zwracamy pochodne równoważne exact-update: dω/dt = (ω_new - ω_old)/dt
    temp.omega_rl = (omega_rl_next - x.omega_rl) / dt;
    temp.omega_rr = (omega_rr_next - x.omega_rr) / dt;

    return temp;
}

} // namespace lem_dynamics_sim_
