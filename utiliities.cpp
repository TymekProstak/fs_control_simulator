#include "uttilities.hpp"

namespace lem_dynamics_sim_ {

State::State() : x(0), y(0), yaw(0), vx(0), vy(0), yaw_rate(0),
                 omega_rr(0), omega_rl(0),
                 delta_left(0), d_delta_left(0), delta_right(0), d_delta_right(0),
                 rack_angle(0), d_rack_angle(0),
                 torque(0), torque_left(0), torque_right(0),
                 fx_rr(0), fx_rl(0), fy_rr(0), fy_rl(0), fy_fr(0), fy_fl(0),
                 prev_ax(0), prev_ay(0) {}

State::State(double value) : 
              x(value), y(value), yaw(value), vx(value), vy(value), yaw_rate(value),
              omega_rr(value), omega_rl(value),
              delta_left(value), d_delta_left(value), delta_right(value), d_delta_right(value),
              rack_angle(value), d_rack_angle(value),
              torque(value), torque_left(value), torque_right(value),
              fx_rr(value), fx_rl(value), fy_rr(value), fy_rl(value), fy_fr(value), fy_fl(value),
              prev_ax(value), prev_ay(value) {}

State::State(const std::vector<double>& values) {
    if (values.size() != 23) {
        throw std::invalid_argument("Vector size must be 23 to initialize State.");
    }
    x = values[0];
    y = values[1];
    yaw = values[2];
    vx = values[3];
    vy = values[4];
    yaw_rate = values[5];
    omega_rr = values[6];
    omega_rl = values[7];
    delta_left = values[8];
    d_delta_left = values[9];
    delta_right = values[10];
    d_delta_right = values[11];
    rack_angle = values[12];
    d_rack_angle = values[13];
    torque = values[14];
    torque_left = values[15];
    torque_right = values[16];
    fx_rr = values[17];
    fx_rl = values[18];
    fy_rr = values[19];
    fy_rl = values[20];
    fy_fr = values[21];
    fy_fl = values[22];
    prev_ax = values[23];
    prev_ay = values[24];
}

void State::setZero() {
    x = y = yaw = vx = vy = yaw_rate = 0.0;
    omega_rr = omega_rl = 0.0;
    delta_left = d_delta_left = delta_right = d_delta_right = 0.0;
    rack_angle = d_rack_angle = 0.0;
    torque = torque_left = torque_right = 0.0;
    fx_rr = fx_rl = fy_rr = fy_rl = fy_fr = fy_fl = 0.0;
    prev_ax = prev_ay = 0.0;
}

State State::operator+(const State& other) const {
    State result;
    result.x = x + other.x;
    result.y = y + other.y;
    result.yaw = yaw + other.yaw;
    result.vx = vx + other.vx;
    result.vy = vy + other.vy;
    result.yaw_rate = yaw_rate + other.yaw_rate;
    result.omega_rr = omega_rr + other.omega_rr;
    result.omega_rl = omega_rl + other.omega_rl;
    result.delta_left = delta_left + other.delta_left;
    result.d_delta_left = d_delta_left + other.d_delta_left;
    result.delta_right = delta_right + other.delta_right;
    result.d_delta_right = d_delta_right + other.d_delta_right;
    result.rack_angle = rack_angle + other.rack_angle;
    result.d_rack_angle = d_rack_angle + other.d_rack_angle;
    result.torque = torque + other.torque;
    result.torque_left = torque_left + other.torque_left;
    result.torque_right = torque_right + other.torque_right;
    result.fx_rr = fx_rr + other.fx_rr;
    result.fx_rl = fx_rl + other.fx_rl;
    result.fy_rr = fy_rr + other.fy_rr;
    result.fy_rl = fy_rl + other.fy_rl;
    result.fy_fr = fy_fr + other.fy_fr;
    result.fy_fl = fy_fl + other.fy_fl;
    result.prev_ax = prev_ax + other.prev_ax;
    result.prev_ay = prev_ay + other.prev_ay;
    return result;
}

State& State::operator+=(const State& other) {
    x += other.x;
    y += other.y;
    yaw += other.yaw;
    vx += other.vx;
    vy += other.vy;
    yaw_rate += other.yaw_rate;
    omega_rr += other.omega_rr;
    omega_rl += other.omega_rl;
    delta_left += other.delta_left;
    d_delta_left += other.d_delta_left;
    delta_right += other.delta_right;
    d_delta_right += other.d_delta_right;
    rack_angle += other.rack_angle;
    d_rack_angle += other.d_rack_angle;
    torque += other.torque;
    torque_left += other.torque_left;
    torque_right += other.torque_right;
    fx_rr += other.fx_rr;
    fx_rl += other.fx_rl;
    fy_rr += other.fy_rr;
    fy_rl += other.fy_rl;
    fy_fr += other.fy_fr;
    fy_fl += other.fy_fl;
    prev_ax += other.prev_ax;
    prev_ay += other.prev_ay;
    return *this;
}

State State::operator*(double scalar) const {
    State result;
    result.x = x * scalar;
    result.y = y * scalar;
    result.yaw = yaw * scalar;
    result.vx = vx * scalar;
    result.vy = vy * scalar;
    result.yaw_rate = yaw_rate * scalar;
    result.omega_rr = omega_rr * scalar;
    result.omega_rl = omega_rl * scalar;
    result.delta_left = delta_left * scalar;
    result.d_delta_left = d_delta_left * scalar;
    result.delta_right = delta_right * scalar;
    result.d_delta_right = d_delta_right * scalar;
    result.rack_angle = rack_angle * scalar;
    result.d_rack_angle = d_rack_angle * scalar;
    result.torque = torque * scalar;
    result.torque_left = torque_left * scalar;
    result.torque_right = torque_right * scalar;
    result.fx_rr = fx_rr * scalar;
    result.fx_rl = fx_rl * scalar;
    result.fy_rr = fy_rr * scalar;
    result.fy_rl = fy_rl * scalar;
    result.fy_fr = fy_fr * scalar;
    result.fy_fl = fy_fl * scalar;
    result.prev_ax = prev_ax * scalar;
    result.prev_ay = prev_ay * scalar;
    return result;
}

State operator*(double scalar, const State& s) {
    return s * scalar;
}

Input::Input() : torque_request(0), rack_angle_request(0) {}

Input::Input(double torque, double rack) : torque_request(torque), rack_angle_request(rack) {}

Input::Input(double value) : torque_request(value), rack_angle_request(value) {}

Input::Input(const std::vector<double>& values) {
    if (values.size() != 2) {
        throw std::invalid_argument("Vector size must be 2 to initialize Input.");
    }
    torque_request = values[0];
    rack_angle_request = values[1];
}

void unwrap_angle(double& angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
}

} // namespace lem_dynamics_sim_