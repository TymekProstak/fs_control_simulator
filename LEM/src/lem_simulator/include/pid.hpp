#pragma once

#include <algorithm> // std::clamp
#include "uttilities.hpp"

namespace lem_dynamics_sim_ {

struct PIDParams {
    double Kp{0.0};
    double Ki{0.0};
    double Kd{0.0};

    double saturation_upper{0.0};
    double saturation_lower{0.0};

    double anti_windup_gain{0.0};
    double leak_time_scale{1.0};
};

class PIDController {
public:
    PIDController() = default;
    explicit PIDController(const PIDParams& params) : params_(params) {}

    void set_params(const PIDParams& params) { params_ = params; }
    void reset() { integrator_ = 0.0; prev_error_ = 0.0; output_ = 0.0; }

    // on_off=false -> "leak" integratora
    void update(double error, double dt, bool on_off = true)
    {
        active = on_off; // aktualizacja flagi aktywności 
        if (dt <= 0.0) return;

        if (!on_off) {
            // leak integrator
            const double tau = (params_.leak_time_scale > 1e-9) ? params_.leak_time_scale : 1e-9;
            integrator_ -= integrator_ * dt / tau;
            return;
        }

        const double P_term = params_.Kp * error;

        integrator_ += error * dt;
        const double I_term = params_.Ki * integrator_;

       // const double D_term = params_.Kd * (error - prev_error_) / dt;

        const double u_unsat = P_term + I_term ; //+ D_term;

        output_ = std::clamp(u_unsat, params_.saturation_lower, params_.saturation_upper);

        // Anti-windup: adjust integrator if output is saturated
        const double u_error = output_ - u_unsat;
        integrator_ += params_.anti_windup_gain * u_error * dt;

        prev_error_ = error;
    }

    double get_output() const { return output_; }

    bool is_active() const { return active; }

private:
    PIDParams params_{};
    double integrator_{0.0};
    double prev_error_{0.0};
    double output_{0.0};
    bool active = false;

};



} // namespace lem_dynamics_sim_



