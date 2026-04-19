#include "EKF/EKF.hpp"

#include <algorithm>
#include <stdexcept>

namespace kalman_filter_kinematic
{

EKF::EKF(const ParamBank& param_bank)
    : param_(param_bank)
{
    reset();
}

void EKF::reset()
{
    state_ = State{};

    x_hat_.setZero();
    P_ = buildInitialCovarianceMatrix();

    imu_ax_comp_ = 0.0;
    imu_ay_comp_ = 0.0;
    imu_yaw_rate_comp_ = 0.0;

    has_imu_ = false;
    has_wheel_speed_ = false;
    has_steer_ = false;
    has_torque_command_ = false;
    has_gravity_init_ = false;

    syncStructFromFilterVector();
}

void EKF::reset(const State& initial_state)
{
    state_ = initial_state;

    syncFilterVectorFromStruct();
    P_ = buildInitialCovarianceMatrix();

    imu_ax_comp_ = 0.0;
    imu_ay_comp_ = 0.0;
    imu_yaw_rate_comp_ = 0.0;

    has_imu_ = false;
    has_wheel_speed_ = false;
    has_steer_ = false;
    has_torque_command_ = false;

    // jeśli wrapper podał g wcześniej przez initial_state, to uznaję że jest
    has_gravity_init_ = (std::abs(state_.g_x_init) > 0.0 ||
                         std::abs(state_.g_y_init) > 0.0 ||
                         std::abs(state_.g_in_plane_modul_init) > 0.0);

    syncStructFromFilterVector();
}

void EKF::predict(double dt)
{
    if (dt <= 0.0) {
        return;
    }

    const VectorX x_prev = x_hat_;
    const MatrixX F = processJacobian(x_prev, dt);
    const MatrixX Qd = buildProcessNoiseMatrix() * dt;

    x_hat_ = processModel(x_prev, dt);
    P_ = F * P_ * F.transpose() + Qd;

    syncStructFromFilterVector();

    if (readBoolParam("kalman_filter_kinematic.logic.enable_debug_pose_integration")) {
        integratePlanarPose(dt);
    }

    updateGravityCompensatedImu();
}

void EKF::setImuMeasurement(double ax_raw, double ay_raw, double yaw_rate_raw)
{
    state_.imu_ax_raw = ax_raw;
    state_.imu_ay_raw = ay_raw;
    state_.imu_yaw_rate_raw = yaw_rate_raw;
    has_imu_ = true;

    updateGravityCompensatedImu();
}

void EKF::setWheelSpeedMeasurement(double vx_meas)
{
    state_.wheel_speed_vx = vx_meas;
    has_wheel_speed_ = true;
}

void EKF::setSteeringMeasurement(double steer, double steer_dot)
{
    state_.steer = steer;
    state_.steer_dot = steer_dot;
    has_steer_ = true;
}

void EKF::setTorqueCommand(double torque_command)
{
    state_.T = torque_command;
    has_torque_command_ = true;
}

void EKF::setInitialGravityInPlane(double gx_init, double gy_init)
{
    state_.g_x_init = gx_init;
    state_.g_y_init = gy_init;
    state_.g_in_plane_modul_init = std::sqrt(gx_init * gx_init + gy_init * gy_init);

    has_gravity_init_ = true;

    updateGravityCompensatedImu();
}

void EKF::updateFromImu()
{
    if (!has_imu_) {
        return;
    }

    updateGravityCompensatedImu();

    {
        RowX H = RowX::Zero();
        H(0, IDX_R) = 1.0;
        scalarMeasurementUpdate(
            imu_yaw_rate_comp_,
            x_hat_(IDX_R),
            H,
            readDoubleParam("kalman_filter_kinematic.measurement_noise.imu_yaw_rate"));
    }

    {
        RowX H = RowX::Zero();
        H(0, IDX_AX) = 1.0;
        scalarMeasurementUpdate(
            imu_ax_comp_,
            x_hat_(IDX_AX),
            H,
            readDoubleParam("kalman_filter_kinematic.measurement_noise.imu_ax"));
    }

    {
        RowX H = RowX::Zero();
        H(0, IDX_AY) = 1.0;
        scalarMeasurementUpdate(
            imu_ay_comp_,
            x_hat_(IDX_AY),
            H,
            readDoubleParam("kalman_filter_kinematic.measurement_noise.imu_ay"));
    }

    syncStructFromFilterVector();
}

void EKF::updateFromWheelSpeed()
{
    if (!has_wheel_speed_) {
        return;
    }

    RowX H = RowX::Zero();
    H(0, IDX_VX) = 1.0;

    scalarMeasurementUpdate(
        state_.wheel_speed_vx,
        x_hat_(IDX_VX),
        H,
        readDoubleParam("kalman_filter_kinematic.measurement_noise.wheel_speed_vx"));

    syncStructFromFilterVector();
}

void EKF::updatePseudoKinematicLateralAccel()
{
    if (!has_steer_) {
        return;
    }

    const double ay_kin =
        computeKinematicLateralAccelExact(x_hat_, state_.steer, state_.steer_dot);

    RowX H = RowX::Zero();
    H(0, IDX_AY) = 1.0;

    scalarMeasurementUpdate(
        ay_kin,
        x_hat_(IDX_AY),
        H,
        readDoubleParam("kalman_filter_kinematic.pseudo_measurement_noise.kinematic_ay"));

    syncStructFromFilterVector();
}

void EKF::updatePseudoTorqueAccel()
{
    if (!has_torque_command_) {
        return;
    }

    const double ax_cmd = computeTorqueBasedLongitudinalAccel(state_.T, x_hat_(IDX_VX));

    RowX H = RowX::Zero();
    H(0, IDX_AX) = 1.0;

    scalarMeasurementUpdate(
        ax_cmd,
        x_hat_(IDX_AX),
        H,
        readDoubleParam("kalman_filter_kinematic.pseudo_measurement_noise.torque_ax"));

    syncStructFromFilterVector();
}
void EKF::updatePseudoZeroLateralVelocity()
{
    const double vx_ref = has_wheel_speed_ ? state_.wheel_speed_vx : x_hat_(IDX_VX);
    const double r = x_hat_(IDX_R);
    const double steer = state_.steer;
    const double torque = state_.T;

    const double vx_thr =
        readDoubleParam("kalman_filter_kinematic.numeric.standstill_vx_threshold");
    const double steer_thr =
        readDoubleParam("kalman_filter_kinematic.numeric.standstill_abs_steer_threshold");
    const double r_thr =
        readDoubleParam("kalman_filter_kinematic.numeric.standstill_abs_r_threshold");
    const double torque_thr =
        readDoubleParam("kalman_filter_kinematic.numeric.standstill_abs_torque_threshold");

    if (std::abs(vx_ref) > vx_thr) {
        return;
    }

    if (std::abs(steer) > steer_thr) {
        return;
    }

    if (std::abs(r) > r_thr) {
        return;
    }

    if (std::abs(torque) > torque_thr) {
        return;
    }

    RowX H = RowX::Zero();
    H(0, IDX_VY) = 1.0;

    scalarMeasurementUpdate(
        0.0,
        x_hat_(IDX_VY),
        H,
        readDoubleParam("kalman_filter_kinematic.pseudo_measurement_noise.zero_vy_standstill"));

    syncStructFromFilterVector();
}

void EKF::integratePlanarPose(double dt)
{
    if (dt <= 0.0) {
        return;
    }

    state_.yaw = wrapAngle(state_.yaw + dt * state_.yaw_rate);

    const double c = std::cos(state_.yaw);
    const double s = std::sin(state_.yaw);

    const double x_dot = state_.vx * c - state_.vy * s;
    const double y_dot = state_.vx * s + state_.vy * c;

    state_.x += dt * x_dot;
    state_.y += dt * y_dot;
}

void EKF::updateGravityCompensatedImu()
{
    imu_yaw_rate_comp_ = state_.imu_yaw_rate_raw;

    if (!readBoolParam("kalman_filter_kinematic.logic.enable_gravity_compensation") || !has_gravity_init_) {
        imu_ax_comp_ = state_.imu_ax_raw;
        imu_ay_comp_ = state_.imu_ay_raw;
        return;
    }

    imu_ax_comp_ = state_.imu_ax_raw - computeGravityBodyX();
    imu_ay_comp_ = state_.imu_ay_raw - computeGravityBodyY();
}

const State& EKF::getState() const
{
    return state_;
}

const EKF::VectorX& EKF::getStateVector() const
{
    return x_hat_;
}

const EKF::MatrixX& EKF::getCovariance() const
{
    return P_;
}

void EKF::syncFilterVectorFromStruct()
{
    x_hat_(IDX_VX) = state_.vx;
    x_hat_(IDX_VY) = state_.vy;
    x_hat_(IDX_R)  = state_.yaw_rate;
    x_hat_(IDX_AX) = state_.ax;
    x_hat_(IDX_AY) = state_.ay;
}

void EKF::syncStructFromFilterVector()
{
    state_.vx       = x_hat_(IDX_VX);
    state_.vy       = x_hat_(IDX_VY);
    state_.yaw_rate = x_hat_(IDX_R);
    state_.ax       = x_hat_(IDX_AX);
    state_.ay       = x_hat_(IDX_AY);
}

EKF::VectorX EKF::processModel(const VectorX& x, double dt) const
{
    VectorX x_next = x;

    const double vx = x(IDX_VX);
    const double vy = x(IDX_VY);
    const double r  = x(IDX_R);
    const double ax = x(IDX_AX);
    const double ay = x(IDX_AY);

    const double vx_dot = ax + r * vy;
    const double vy_dot = ay - r * vx;
    const double r_dot  = computeKinematicYawRateDotExact(x, state_.steer, state_.steer_dot);

    x_next(IDX_VX) = vx + dt * vx_dot;
    x_next(IDX_VY) = vy + dt * vy_dot;
    x_next(IDX_R)  = r  + dt * r_dot;
    x_next(IDX_AX) = ax;
    x_next(IDX_AY) = ay;

    return x_next;
}

EKF::MatrixX EKF::processJacobian(const VectorX& x, double dt) const
{
    MatrixX F = MatrixX::Identity();

    const double vx = x(IDX_VX);
    const double vy = x(IDX_VY);
    const double r  = x(IDX_R);

    F(IDX_VX, IDX_VY) = dt * r;
    F(IDX_VX, IDX_R)  = dt * vy;
    F(IDX_VX, IDX_AX) = dt;

    F(IDX_VY, IDX_VX) = -dt * r;
    F(IDX_VY, IDX_R)  = -dt * vx;
    F(IDX_VY, IDX_AY) = dt;

    const double L = readDoubleParam("kalman_filter_kinematic.vehicle.wheelbase");
    const double min_vx = readDoubleParam("kalman_filter_kinematic.numeric.min_vx_for_kinematic_model");
    const double max_abs_steer = readDoubleParam("kalman_filter_kinematic.numeric.max_abs_steer_for_linear_region");

    const double delta = std::clamp(state_.steer, -max_abs_steer, max_abs_steer);
    const double tan_delta = std::tan(delta);
    const double sec2_delta = 1.0 / (std::cos(delta) * std::cos(delta));

    const double dvxeff_dvx = (std::abs(vx) >= min_vx) ? 1.0 : 0.0;

    F(IDX_R, IDX_VX) += dt * ((sec2_delta / L) * state_.steer_dot * dvxeff_dvx);
    F(IDX_R, IDX_VY) += dt * ((tan_delta / L) * r);
    F(IDX_R, IDX_R ) += dt * ((tan_delta / L) * vy);
    F(IDX_R, IDX_AX) += dt * (tan_delta / L);

    return F;
}

void EKF::scalarMeasurementUpdate(double z,
                                  double h,
                                  const RowX& H,
                                  double R_scalar)
{
    const double S = (H * P_ * H.transpose())(0, 0) + R_scalar;

    if (S <= 1e-12) {
        return;
    }

    const Eigen::Matrix<double, NX, 1> K = P_ * H.transpose() / S;
    const double innovation = z - h;

    x_hat_ = x_hat_ + K * innovation;

    const MatrixX I = MatrixX::Identity();
    const MatrixX KH = K * H;
    P_ = (I - KH) * P_ * (I - KH).transpose() + K * R_scalar * K.transpose();
}

double EKF::computeKinematicYawRate(double vx, double steer) const
{
    const double L = readDoubleParam("kalman_filter_kinematic.vehicle.wheelbase");
    const double min_vx = readDoubleParam("kalman_filter_kinematic.numeric.min_vx_for_kinematic_model");
    const double max_abs_steer = readDoubleParam("kalman_filter_kinematic.numeric.max_abs_steer_for_linear_region");

    const double vx_eff =
        (std::abs(vx) < min_vx) ? ((vx >= 0.0) ? min_vx : -min_vx) : vx;

    const double steer_clamped = std::clamp(steer, -max_abs_steer, max_abs_steer);

    return (vx_eff / L) * std::tan(steer_clamped);
}

double EKF::computeKinematicYawRateDotExact(const VectorX& x,
                                            double steer,
                                            double steer_dot) const
{
    const double L = readDoubleParam("kalman_filter_kinematic.vehicle.wheelbase");
    const double min_vx = readDoubleParam("kalman_filter_kinematic.numeric.min_vx_for_kinematic_model");
    const double max_abs_steer = readDoubleParam("kalman_filter_kinematic.numeric.max_abs_steer_for_linear_region");

    const double vx = x(IDX_VX);
    const double vy = x(IDX_VY);
    const double r  = x(IDX_R);
    const double ax = x(IDX_AX);

    const double vx_eff =
        (std::abs(vx) < min_vx) ? ((vx >= 0.0) ? min_vx : -min_vx) : vx;

    const double delta = std::clamp(steer, -max_abs_steer, max_abs_steer);

    const double tan_delta = std::tan(delta);
    const double sec2_delta = 1.0 / (std::cos(delta) * std::cos(delta));

    const double vx_dot = ax + r * vy;

    return (tan_delta / L) * vx_dot + (vx_eff / L) * sec2_delta * steer_dot;
}

double EKF::computeKinematicLateralAccelExact(const VectorX& x,
                                              double steer,
                                              double steer_dot) const
{
    const double lr = readDoubleParam("kalman_filter_kinematic.vehicle.lr");
    const double min_vx = readDoubleParam("kalman_filter_kinematic.numeric.min_vx_for_kinematic_model");

    const double vx = x(IDX_VX);

    const double vx_eff =
        (std::abs(vx) < min_vx) ? ((vx >= 0.0) ? min_vx : -min_vx) : vx;

    const double r_kin = computeKinematicYawRate(vx, steer);
    const double r_kin_dot = computeKinematicYawRateDotExact(x, steer, steer_dot);

    const double vy_kin_dot = lr * r_kin_dot;
    const double ay_kin = vy_kin_dot + vx_eff * r_kin;

    return ay_kin;
}

double EKF::computeTorqueBasedLongitudinalAccel(double torque_cmd, double vx) const
{
    (void)vx;

    const double ax_gain = readDoubleParam("kalman_filter_kinematic.torque_model.ax_gain");
    const double ax_bias = readDoubleParam("kalman_filter_kinematic.torque_model.ax_bias");

    return ax_gain * torque_cmd + ax_bias;
}

double EKF::computeGravityBodyX() const
{
    if (!has_gravity_init_) {
        return 0.0;
    }

    const double psi = state_.yaw;
    const double c = std::cos(psi);
    const double s = std::sin(psi);

    return c * state_.g_x_init + s * state_.g_y_init;
}

double EKF::computeGravityBodyY() const
{
    if (!has_gravity_init_) {
        return 0.0;
    }

    const double psi = state_.yaw;
    const double c = std::cos(psi);
    const double s = std::sin(psi);

    return -s * state_.g_x_init + c * state_.g_y_init;
}

double EKF::readDoubleParam(const std::string& key) const
{
    return param_.get(key);
}

int EKF::readIntParam(const std::string& key) const
{
    return static_cast<int>(std::lround(param_.get(key)));
}

bool EKF::readBoolParam(const std::string& key) const
{
    return param_.get(key) > 0.5;
}

EKF::MatrixX EKF::buildProcessNoiseMatrix() const
{
    MatrixX Q = MatrixX::Zero();

    Q(IDX_VX, IDX_VX) = readDoubleParam("kalman_filter_kinematic.process_noise.vx");
    Q(IDX_VY, IDX_VY) = readDoubleParam("kalman_filter_kinematic.process_noise.vy");
    Q(IDX_R,  IDX_R ) = readDoubleParam("kalman_filter_kinematic.process_noise.r");
    Q(IDX_AX, IDX_AX) = readDoubleParam("kalman_filter_kinematic.process_noise.ax");
    Q(IDX_AY, IDX_AY) = readDoubleParam("kalman_filter_kinematic.process_noise.ay");

    return Q;
}

EKF::MatrixX EKF::buildInitialCovarianceMatrix() const
{
    MatrixX P0 = MatrixX::Zero();

    P0(IDX_VX, IDX_VX) = readDoubleParam("kalman_filter_kinematic.initial_covariance.vx");
    P0(IDX_VY, IDX_VY) = readDoubleParam("kalman_filter_kinematic.initial_covariance.vy");
    P0(IDX_R,  IDX_R ) = readDoubleParam("kalman_filter_kinematic.initial_covariance.r");
    P0(IDX_AX, IDX_AX) = readDoubleParam("kalman_filter_kinematic.initial_covariance.ax");
    P0(IDX_AY, IDX_AY) = readDoubleParam("kalman_filter_kinematic.initial_covariance.ay");

    return P0;
}

double EKF::wrapAngle(double angle)
{
    constexpr double PI = 3.14159265358979323846;

    while (angle > PI)  angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

} // namespace kalman_filter_kinematic