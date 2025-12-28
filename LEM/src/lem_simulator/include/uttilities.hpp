#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <functional>
#include <cmath>
#include <algorithm>

namespace lem_dynamics_sim_ {

struct State {
    double x, y, yaw, vx, vy, yaw_rate;
    double omega_rr, omega_rl;
    double delta_left, d_delta_left, delta_right, d_delta_right;
    double rack_angle, d_rack_angle;
    double torque, torque_left, torque_right;
    double fx_rr, fx_rl, fy_rr, fy_rl, fy_fr, fy_fl;
    double prev_ax, prev_ay;

    State();
    explicit State(double value);
    explicit State(const std::vector<double>& values);

    void setZero();

    State operator+(const State& other) const;
    State& operator+=(const State& other);
    State operator*(double scalar) const;
    friend State operator*(double scalar, const State& s);
};

struct Input {
    double torque_request, rack_angle_request;

    Input();
    Input(double torque, double rack);
    explicit Input(double value);
    explicit Input(const std::vector<double>& values);
};

void unwrap_angle(double& angle);

} // namespace lem_dynamics_sim_