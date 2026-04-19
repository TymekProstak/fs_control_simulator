#pragma once

#include <iostream>
#include <string>
#include <cmath>
#include <Eigen/Dense>

#include "EKF/ParamBank.hpp"

namespace kalman_filter_kinematic
{

struct State
{
    // pomocnicza integracja do debug / inline case
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;

    // główne stany EKF v1.5
    double vx = 0.0;
    double vy = 0.0;
    double yaw_rate = 0.0;
    double ax = 0.0;
    double ay = 0.0;

    // sygnały z callbacków / wejścia
    double steer = 0.0;
    double steer_dot = 0.0;
    double T = 0.0;

    // ostatnie surowe pomiary
    double imu_ax_raw = 0.0;
    double imu_ay_raw = 0.0;
    double imu_yaw_rate_raw = 0.0;
    double wheel_speed_vx = 0.0;

    // początkowa składowa g w płaszczyźnie body
    double g_x_init = 0.0;
    double g_y_init = 0.0;
    double g_in_plane_modul_init = 0.0;
};

class EKF
{
public:
    static constexpr int NX = 5;

    enum StateIndex : int
    {
        IDX_VX = 0,
        IDX_VY = 1,
        IDX_R  = 2,
        IDX_AX = 3,
        IDX_AY = 4
    };

    using VectorX = Eigen::Matrix<double, NX, 1>;
    using MatrixX = Eigen::Matrix<double, NX, NX>;
    using RowX    = Eigen::Matrix<double, 1, NX>;

public:
    explicit EKF(const ParamBank& param_bank);

    // lifecycle
    void reset();
    void reset(const State& initial_state);

    // prediction
    void predict(double dt);

    // raw inputs / callbacks
    void setImuMeasurement(double ax_raw, double ay_raw, double yaw_rate_raw);
    void setWheelSpeedMeasurement(double vx_meas);
    void setSteeringMeasurement(double steer, double steer_dot = 0.0);
    void setTorqueCommand(double torque_command);

    // wrapper-provided gravity initialization
    void setInitialGravityInPlane(double gx_init, double gy_init);

    // true measurements
    void updateFromImu();
    void updateFromWheelSpeed();

    // pseudo-measurements
    void updatePseudoKinematicLateralAccel();
    void updatePseudoTorqueAccel();
    void updatePseudoZeroLateralVelocity();
    
    // helpers
    void integratePlanarPose(double dt);
    void updateGravityCompensatedImu();

    // getters
    const State& getState() const;
    const VectorX& getStateVector() const;
    const MatrixX& getCovariance() const;

private:
    // sync
    void syncFilterVectorFromStruct();
    void syncStructFromFilterVector();

    // process model
    VectorX processModel(const VectorX& x, double dt) const;
    MatrixX processJacobian(const VectorX& x, double dt) const;

    // generic scalar EKF update
    void scalarMeasurementUpdate(double z,
                                 double h,
                                 const RowX& H,
                                 double R_scalar);

    // model helpers
    double computeKinematicYawRate(double vx, double steer) const;
    double computeKinematicYawRateDotExact(const VectorX& x,
                                           double steer,
                                           double steer_dot) const;
    double computeKinematicLateralAccelExact(const VectorX& x,
                                             double steer,
                                             double steer_dot) const;
    double computeTorqueBasedLongitudinalAccel(double torque_cmd, double vx) const;

    // gravity compensation helpers
    double computeGravityBodyX() const;
    double computeGravityBodyY() const;

    // ParamBank access helpers
    double readDoubleParam(const std::string& key) const;
    int readIntParam(const std::string& key) const;
    bool readBoolParam(const std::string& key) const;

    // covariance builders from ParamBank
    MatrixX buildProcessNoiseMatrix() const;
    MatrixX buildInitialCovarianceMatrix() const;

    // misc
    static double wrapAngle(double angle);

private:
    State state_{};

    ParamBank param_;

    VectorX x_hat_ = VectorX::Zero();
    MatrixX P_ = MatrixX::Identity();

    // ostatnie IMU po kompensacji g
    double imu_ax_comp_ = 0.0;
    double imu_ay_comp_ = 0.0;
    double imu_yaw_rate_comp_ = 0.0;

    // availability flags
    bool has_imu_ = false;
    bool has_wheel_speed_ = false;
    bool has_steer_ = false;
    bool has_torque_command_ = false;
    bool has_gravity_init_ = false;
};

} // namespace kalman_filter_kinematic