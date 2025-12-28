/// ===============================
// mpc_interface.cpp (SPLIT-V version)
// - v_path (z planera) używam TYLKO w epsi_dot: r - kappa*v_path
// - v_vehicle (z auta) używam w ey_dot, slip/atan2 oraz w dynamice bocznej (np. -v_vehicle*r)
// - v_vehicle[k] predykuję z vx0_body (odom) + integracja a_long_ref (z procesora ścieżki)
// - V_SAFE używany TYLKO do mianownika atan2 (slip safety)
// ===============================

#include "mpc_interface.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <unsupported/Eigen/MatrixFunctions>

#include <sstream>
#include <iomanip>
#include <limits>

extern "C" {
    #include "acados_solver_mpc_ltv_discrete.h"
}

namespace v2_control {

// ============================================================
// Safety for slip computations only
// ============================================================
static constexpr double V_SAFE = 1.0;

static inline double v_safe_for_slip(double v_vehicle)
{
    const double vmag = std::abs(v_vehicle);
    return (vmag < V_SAFE) ? V_SAFE : vmag;
}

// ============================================================
// DIAG helpers
// ============================================================
static inline bool eigen_all_finite(const Eigen::MatrixXd& M) {
    return (M.array().isFinite()).all();
}
static inline bool eigen_all_finite_vec(const Eigen::VectorXd& v) {
    return (v.array().isFinite()).all();
}

static inline void dump_acados_solver_stats(ocp_nlp_solver* nlp_solver, int status)
{
    int sqp_iter = -1;
    double time_tot   = std::numeric_limits<double>::quiet_NaN();
    double cost_value = std::numeric_limits<double>::quiet_NaN();

    ocp_nlp_get(nlp_solver, "sqp_iter",   &sqp_iter);
    ocp_nlp_get(nlp_solver, "time_tot",   &time_tot);
    ocp_nlp_get(nlp_solver, "cost_value", &cost_value);

    ROS_WARN_STREAM(std::fixed << std::setprecision(6)
        << "[MPC DIAG] status=" << status
        << " | sqp_iter=" << sqp_iter
        << " | time_tot=" << time_tot
        << " | cost=" << cost_value
    );
}

static inline void dump_solution_preview(ocp_nlp_config* nlp_config,
                                         ocp_nlp_dims* nlp_dims,
                                         ocp_nlp_out* nlp_out,
                                         int N_hor,
                                         int max_k)
{
    max_k = std::min(max_k, N_hor);
    double xk[NX];
    double uk[NU];

    for (int k = 0; k <= max_k; ++k) {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "x", xk);
        if (k < N_hor) ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "u", uk);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);
        oss << "[MPC DIAG] k=" << k << " x=[";
        for (int i = 0; i < NX; ++i) {
            oss << xk[i];
            if (i + 1 < NX) oss << ", ";
        }
        oss << "]";
        if (k < N_hor) oss << " u=[" << uk[0] << "]";
        ROS_WARN_STREAM(oss.str());
    }
}

static inline void dump_solution_maxima(ocp_nlp_config* nlp_config,
                                       ocp_nlp_dims* nlp_dims,
                                       ocp_nlp_out* nlp_out,
                                       int N_hor)
{
    double xk[NX];
    double uk[NU];

    double max_abs_u = 0.0;
    double max_abs_ey = 0.0;
    double max_abs_epsi = 0.0;
    double max_abs_vy = 0.0;
    double max_abs_r = 0.0;
    double max_abs_delta = 0.0;
    double max_abs_d_delta = 0.0;
    double max_abs_d_req = 0.0;

    for (int k = 0; k <= N_hor; ++k) {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "x", xk);

        max_abs_ey      = std::max(max_abs_ey,      std::abs(xk[0]));
        max_abs_epsi    = std::max(max_abs_epsi,    std::abs(xk[1]));
        max_abs_vy      = std::max(max_abs_vy,      std::abs(xk[2]));
        max_abs_r       = std::max(max_abs_r,       std::abs(xk[3]));
        max_abs_delta   = std::max(max_abs_delta,   std::abs(xk[4]));
        max_abs_d_delta = std::max(max_abs_d_delta, std::abs(xk[5]));
        max_abs_d_req   = std::max(max_abs_d_req,   std::abs(xk[6]));

        if (k < N_hor) {
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "u", uk);
            max_abs_u = std::max(max_abs_u, std::abs(uk[0]));
        }
    }

    ROS_WARN_STREAM(std::fixed << std::setprecision(6)
        << "[MPC DIAG] maxima over horizon:"
        << " max|u|=" << max_abs_u
        << " max|ey|=" << max_abs_ey
        << " max|epsi|=" << max_abs_epsi
        << " max|vy|=" << max_abs_vy
        << " max|r|=" << max_abs_r
        << " max|delta|=" << max_abs_delta
        << " max|d_delta|=" << max_abs_d_delta
        << " max|d_req|=" << max_abs_d_req
    );
}

// ============================================================
// exact discretization via expm: [A B; 0 0]
// ============================================================
static inline void discretize_expm_AB(
    const Eigen::Matrix<double, NX, NX>& A,
    const Eigen::Matrix<double, NX, NU>& B,
    double dt,
    Eigen::Matrix<double, NX, NX>& Ad,
    Eigen::Matrix<double, NX, NU>& Bd)
{
    Eigen::Matrix<double, NX + NU, NX + NU> M = Eigen::Matrix<double, NX + NU, NX + NU>::Zero();
    M.block<NX, NX>(0, 0)  = A;
    M.block<NX, NU>(0, NX) = B;

    Eigen::Matrix<double, NX + NU, NX + NU> E = (M * dt).exp();
    Ad = E.block<NX, NX>(0, 0);
    Bd = E.block<NX, NU>(0, NX);
}

// ============================================================
// Continuous dynamics (atan2 slip) — SPLIT V
// v_path    -> tylko do epsi_dot = r - kappa*v_path
// v_vehicle -> ey_dot, slip, -v_vehicle*r, itd.
// ============================================================
static inline Eigen::Matrix<double, NX, 1> f_splitv_continuous(
    const Eigen::Matrix<double, NX, 1>& x,
    double u,
    double kappa,
    double v_path,
    double v_vehicle,
    const ParamBank& param)
{
    const double m  = param.get("model_m");
    const double Iz = param.get("model_Iz");
    const double lf = param.get("model_lf");
    const double lr = param.get("model_lr");
    const double Cr = param.get("model_Cr");
    const double Cf = param.get("model_Cf");

    const double omega    = param.get("model_steer_natural_freq");
    const double damp     = param.get("model_steer_damping");
    const double omega_sq = omega * omega;

    const double v_slip = v_safe_for_slip(v_vehicle);

    const double epsi    = x(1);
    const double vy      = x(2);
    const double r       = x(3);
    const double delta   = x(4);
    const double d_delta = x(5);
    const double d_req   = x(6);

    const double s_epsi  = std::sin(epsi);
    const double c_epsi  = std::cos(epsi);
    const double c_delta = std::cos(delta);

    const double yf = vy + lf * r;
    const double yr = vy - lr * r;

    const double alpha_f = delta - std::atan2(yf, v_slip);
    const double alpha_r = -std::atan2(yr, v_slip);

    const double Fyf = Cf * alpha_f;
    const double Fyr = Cr * alpha_r;

    Eigen::Matrix<double, NX, 1> xdot;
    xdot.setZero();

    xdot(0) = vy * c_epsi + v_vehicle * s_epsi;   // ey_dot
    xdot(1) = r - kappa * v_path;                 // epsi_dot

    xdot(2) = (Fyf * c_delta + Fyr) / m - v_vehicle * r; // vy_dot
    xdot(3) = (lf * Fyf * c_delta - lr * Fyr) / Iz;      // r_dot

    xdot(4) = d_delta;
    xdot(5) = -omega_sq * delta - 2.0 * damp * omega * d_delta + omega_sq * d_req;
    xdot(6) = u;

    return xdot;
}

static inline Eigen::Matrix<double, NX, 1> rk4_step_splitv(
    const Eigen::Matrix<double, NX, 1>& x,
    double u,
    double kappa,
    double v_path,
    double v_vehicle,
    double dt,
    const ParamBank& param)
{
    const auto k1 = f_splitv_continuous(x,                 u, kappa, v_path, v_vehicle, param);
    const auto k2 = f_splitv_continuous(x + 0.5*dt*k1,     u, kappa, v_path, v_vehicle, param);
    const auto k3 = f_splitv_continuous(x + 0.5*dt*k2,     u, kappa, v_path, v_vehicle, param);
    const auto k4 = f_splitv_continuous(x + dt*k3,         u, kappa, v_path, v_vehicle, param);
    return x + (dt/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

// ============================================================
// Helper: a_long fallback from v_path (finite difference)
// ============================================================
static inline std::vector<double> build_a_long_from_v_path(
    int N_hor,
    double dt,
    const std::vector<double>& v_path)
{
    std::vector<double> a;
    a.assign(N_hor, 0.0);

    if ((int)v_path.size() < 2 || dt <= 0.0) return a;

    for (int k = 0; k < N_hor; ++k) {
        const int k0 = std::min(k, (int)v_path.size() - 2);
        const int k1 = k0 + 1;
        const double dv = v_path[k1] - v_path[k0];
        double ak = dv / dt;
        if (!std::isfinite(ak)) ak = 0.0;
        a[k] = ak;
    }
    return a;
}

// ============================================================
// Helper: build predicted v_vehicle[k] from vx0 + a_long_ref
// ============================================================
static inline std::vector<double> build_v_vehicle_pred(
    int N_hor,
    double dt,
    double vx0_body,
    const std::vector<double>& a_long_ref)
{
    std::vector<double> vveh;
    vveh.resize(N_hor);

    double v = std::max(0.0, vx0_body);

    for (int k = 0; k < N_hor; ++k) {
        vveh[k] = std::max(0.0, v);

        if (k < (int)a_long_ref.size()) {
            double a = a_long_ref[k];
            if (!std::isfinite(a)) a = 0.0;
            v += a * dt;
        }
    }
    return vveh;
}

// ============================================================
// Constructors / Destructor
// ============================================================
MPCInterface::MPCInterface()
{
    std::cout << "[MPCInterface] DEFAULT ctor – solver NOT initialized properly!" << std::endl;
    capsule_    = nullptr;
    nlp_config_ = nullptr;
    nlp_dims_   = nullptr;
    nlp_in_     = nullptr;
    nlp_out_    = nullptr;
    nlp_solver_ = nullptr;
    is_initialized_ = false;
    last_output.assign(N, 0.0);
}

MPCInterface::MPCInterface(const ParamBank &P)
{
    ROS_INFO("[MPCInterface] Constructing MPCInterface with ParamBank");

    capsule_ = mpc_ltv_discrete_acados_create_capsule();
    if (!capsule_) {
        ROS_ERROR("[MPC SOLVER] create_capsule returned NULL!");
        return;
    }

    const int status = mpc_ltv_discrete_acados_create(capsule_);
    if (status != 0) {
        ROS_ERROR_STREAM("[MPC SOLVER] Could not create ACADOS solver! Status: " << status);
    } else {
        nlp_config_ = mpc_ltv_discrete_acados_get_nlp_config(capsule_);
        nlp_dims_   = mpc_ltv_discrete_acados_get_nlp_dims(capsule_);
        nlp_in_     = mpc_ltv_discrete_acados_get_nlp_in(capsule_);
        nlp_out_    = mpc_ltv_discrete_acados_get_nlp_out(capsule_);
        nlp_solver_ = mpc_ltv_discrete_acados_get_nlp_solver(capsule_);
        ROS_INFO_STREAM("[MPC SOLVER] Solver created successfully.");
    }

    param_ = P;
    is_initialized_ = false;
    last_output.assign(N, 0.0);
}

MPCInterface::~MPCInterface()
{
    ROS_INFO("[MPCInterface] Destructor called");
    if (capsule_) {
        ROS_INFO("[MPCInterface] Freeing ACADOS solver and capsule");
        mpc_ltv_discrete_acados_free(capsule_);
        mpc_ltv_discrete_acados_free_capsule(capsule_);
        capsule_ = nullptr;
    }
}

// ============================================================
// reset_initial_guess — SPLIT-V
// - do ey_pred używam v_vehicle_pred[i]
// ============================================================
void MPCInterface::reset_initial_guess_splitv_(const MPC_State& x0,
                                              const std::vector<double>& v_vehicle_pred)
{
    ROS_INFO("[MPCInterface] Resetting initial guess (Straight Line Prediction, split-v)");
    if (!nlp_config_ || !nlp_dims_ || !nlp_in_ || !nlp_out_) return;

    const double dt = 1.0 / param_.get("odom_frequency");
    const double v_fallback = param_.get("v_target");

    const double current_epsi = x0.epsi;

    double x_traj[NX];
    double u_zero[NU] = {0.0};

    double ey_pred = x0.ey;

    for (int i = 0; i <= N; ++i) {
        double v_i = v_fallback;
        if (!v_vehicle_pred.empty()) {
            const int idx = std::min(i, (int)v_vehicle_pred.size() - 1);
            if (std::isfinite(v_vehicle_pred[idx])) v_i = v_vehicle_pred[idx];
        }

        if (i > 0) {
            ey_pred += (v_i * std::sin(current_epsi)) * dt;
        }

        x_traj[0] = ey_pred;
        x_traj[1] = current_epsi;
        x_traj[2] = 0.0;
        x_traj[3] = 0.0;
        x_traj[4] = 0.0;
        x_traj[5] = 0.0;
        x_traj[6] = 0.0;

        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", x_traj);
        if (i < N) ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", u_zero);
    }
}

// ============================================================
// Jacobian — SPLIT-V
// ============================================================
void MPCInterface::calculate_continuous_jacobian_splitv_(
    const Eigen::Matrix<double, NX, 1>& x,
    double /*v_path*/,
    double v_vehicle,
    Eigen::Matrix<double, NX, NX>& Ac,
    Eigen::Matrix<double, NX, NU>& Bc)
{
    const double m  = param_.get("model_m");
    const double Iz = param_.get("model_Iz");
    const double lf = param_.get("model_lf");
    const double lr = param_.get("model_lr");
    const double Cr = param_.get("model_Cr");
    const double Cf = param_.get("model_Cf");

    const double omega    = param_.get("model_steer_natural_freq");
    const double damp     = param_.get("model_steer_damping");
    const double omega_sq = omega * omega;

    const double v_slip = v_safe_for_slip(v_vehicle);

    const double epsi  = x(1);
    const double vy    = x(2);
    const double r     = x(3);
    const double delta = x(4);

    const double s_epsi  = std::sin(epsi);
    const double c_epsi  = std::cos(epsi);
    const double s_delta = std::sin(delta);
    const double c_delta = std::cos(delta);

    const double yf = vy + lf * r;
    const double yr = vy - lr * r;

    const double alpha_f = delta - std::atan2(yf, v_slip);
    const double alpha_r = -std::atan2(yr, v_slip);

    const double Fyf = Cf * alpha_f;
    const double Fyr = Cr * alpha_r;

    const double denom_f = v_slip*v_slip + yf*yf;
    const double denom_r = v_slip*v_slip + yr*yr;

    const double datan_f_dyf = v_slip / denom_f;
    const double datan_r_dyr = v_slip / denom_r;

    const double d_alpha_f_d_vy    = -datan_f_dyf;
    const double d_alpha_f_d_r     = -datan_f_dyf * lf;
    const double d_alpha_f_d_delta =  1.0;

    const double d_alpha_r_d_vy = -datan_r_dyr;
    const double d_alpha_r_d_r  =  datan_r_dyr * lr;

    const double dFyf_d_vy    = Cf * d_alpha_f_d_vy;
    const double dFyf_d_r     = Cf * d_alpha_f_d_r;
    const double dFyf_d_delta = Cf * d_alpha_f_d_delta;

    const double dFyr_d_vy = Cr * d_alpha_r_d_vy;
    const double dFyr_d_r  = Cr * d_alpha_r_d_r;

    Ac.setZero();
    Bc.setZero();

    // ey_dot = vy*cos(epsi) + v_vehicle*sin(epsi)
    Ac(0, 1) = -vy * s_epsi + v_vehicle * c_epsi;
    Ac(0, 2) =  c_epsi;

    // epsi_dot = r - kappa*v_path -> zależy od r
    Ac(1, 3) = 1.0;

    // vy_dot = (Fyf*cos(delta)+Fyr)/m - v_vehicle*r
    Ac(2, 2) = (1.0 / m) * (dFyf_d_vy * c_delta + dFyr_d_vy);
    Ac(2, 3) = (1.0 / m) * (dFyf_d_r  * c_delta + dFyr_d_r) - v_vehicle;

    const double d_vy_term_d_delta = dFyf_d_delta * c_delta - Fyf * s_delta;
    Ac(2, 4) = (1.0 / m) * d_vy_term_d_delta;

    // r_dot = (lf*Fyf*cos(delta) - lr*Fyr)/Iz
    Ac(3, 2) = (1.0 / Iz) * (lf * dFyf_d_vy * c_delta - lr * dFyr_d_vy);
    Ac(3, 3) = (1.0 / Iz) * (lf * dFyf_d_r  * c_delta - lr * dFyr_d_r);

    const double d_r_term_d_delta = lf * (dFyf_d_delta * c_delta - Fyf * s_delta);
    Ac(3, 4) = (1.0 / Iz) * d_r_term_d_delta;

    // actuator states
    Ac(4, 5) = 1.0;
    Ac(5, 4) = -omega_sq;
    Ac(5, 5) = -2.0 * damp * omega;
    Ac(5, 6) =  omega_sq;

    // u = d(d_req)/dt
    Bc(6, 0) = 1.0;
}

// ============================================================
// LTV matrices — SPLIT-V
// ============================================================
void MPCInterface::ltv_matrixes_to_acados_splitv_(const MPC_State& x0,
                                                 const std::vector<double>& kappa_vec,
                                                 const std::vector<double>& v_path_vec,
                                                 const std::vector<double>& v_vehicle_vec)
{
    const double dt = 1.0 / param_.get("odom_frequency");
    const double v_path_fallback = param_.get("v_target");
    const double v_vehicle_fallback = param_.get("v_target");

    std::vector<Eigen::Matrix<double, NX, 1>> x_traj(N + 1);
    x_traj[0] << x0.ey, x0.epsi, x0.vy, x0.r, x0.delta, x0.d_delta, x0.delta_request;

    for (int k = 0; k < N; ++k)
    {
        double u_k = 0.0;
        if (!last_output.empty()) {
            int idx = std::min(k, (int)last_output.size() - 1);
            u_k = last_output[idx];
        }

        const double kappa =
            (k < (int)kappa_vec.size()) ? kappa_vec[k]
                                        : (kappa_vec.empty() ? 0.0 : kappa_vec.back());

        double v_path = v_path_fallback;
        if (!v_path_vec.empty()) {
            const int vidx = std::min(k, (int)v_path_vec.size() - 1);
            if (std::isfinite(v_path_vec[vidx])) v_path = v_path_vec[vidx];
        }

        double v_vehicle = v_vehicle_fallback;
        if (!v_vehicle_vec.empty()) {
            const int vidx = std::min(k, (int)v_vehicle_vec.size() - 1);
            if (std::isfinite(v_vehicle_vec[vidx])) v_vehicle = v_vehicle_vec[vidx];
        }
        v_vehicle = std::max(0.0, v_vehicle);

        const Eigen::Matrix<double, NX, 1> x_curr = x_traj[k];

        Eigen::Matrix<double, NX, NX> Ac_cont;
        Eigen::Matrix<double, NX, NU> Bc_cont;
        calculate_continuous_jacobian_splitv_(x_curr, v_path, v_vehicle, Ac_cont, Bc_cont);

        Eigen::Matrix<double, NX, NX> Ad;
        Eigen::Matrix<double, NX, NU> Bd;
        discretize_expm_AB(Ac_cont, Bc_cont, dt, Ad, Bd);

        const Eigen::Matrix<double, NX, 1> x_next =
            rk4_step_splitv(x_curr, u_k, kappa, v_path, v_vehicle, dt, param_);
        x_traj[k + 1] = x_next;

        Eigen::Matrix<double, NX, 1> Kd = x_next - (Ad * x_curr + Bd * u_k);

        {
            Eigen::MatrixXd Ad_d = Ad;
            Eigen::MatrixXd Bd_d = Bd;
            Eigen::VectorXd Kd_d = Kd;
            Eigen::VectorXd xn_d = x_next;
            Eigen::VectorXd xc_d = x_curr;

            if (!eigen_all_finite(Ad_d) || !eigen_all_finite(Bd_d) ||
                !eigen_all_finite_vec(Kd_d) || !eigen_all_finite_vec(xn_d) || !eigen_all_finite_vec(xc_d))
            {
                ROS_ERROR_STREAM("[MPC DIAG] NaN/INF in LTV params at stage k=" << k
                    << " | kappa=" << kappa
                    << " | u_k=" << u_k
                    << " | v_path=" << v_path
                    << " | v_vehicle=" << v_vehicle
                    << " | x_curr=" << x_curr.transpose());

                ROS_ERROR_STREAM("[MPC DIAG] norms: ||Ad||=" << Ad.norm()
                    << " ||Bd||=" << Bd.norm()
                    << " ||Kd||=" << Kd.norm()
                    << " ||x_next||=" << x_next.norm());
            }
        }

        std::vector<double> p_vec;
        p_vec.reserve(NX*NX + NX*NU + NX);

        for (int c = 0; c < NX; ++c) for (int r0 = 0; r0 < NX; ++r0) p_vec.push_back(Ad(r0, c));
        for (int c = 0; c < NU; ++c) for (int r0 = 0; r0 < NX; ++r0) p_vec.push_back(Bd(r0, c));
        for (int r0 = 0; r0 < NX; ++r0) p_vec.push_back(Kd(r0));

        mpc_ltv_discrete_acados_update_params(capsule_, k, p_vec.data(), (int)p_vec.size());
    }
}

// ============================================================
// Costs (unchanged)
// ============================================================
void MPCInterface::set_cost_to_acados()
{
    if (!nlp_config_ || !nlp_dims_ || !nlp_in_) return;

    const double Q_y      = param_.get("mpc_cost_Q_y");
    const double Q_psi    = param_.get("mpc_cost_Q_psi");
    const double R_ddelta = param_.get("mpc_cost_R_ddelta");
    const double Q_r      = param_.get("mpc_cost_Q_r");
    const double Q_delta  = param_.get("mpc_cost_Q_delta");

    const double term_scale = 1.0;

    const int ny = NX + NU;
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(ny, ny);
    W(0, 0)     = Q_y;
    W(1, 1)     = Q_psi;
    W(3, 3)     = Q_r;
    W(4, 4)     = Q_delta;
    W(NX, NX)   = R_ddelta;

    const int ny_e = NX;
    Eigen::MatrixXd W_e = Eigen::MatrixXd::Zero(ny_e, ny_e);
    W_e(0, 0) = Q_y   * term_scale;
    W_e(1, 1) = Q_psi * term_scale;

    for (int i = 0; i < N; ++i) ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "W", W.data());
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N, "W", W_e.data());
}

// ============================================================
// SOLVE — SPLIT-V (pełna wersja)
// ============================================================
MPC_Return MPCInterface::solve(const MPC_State &x0,
                               const Eigen::VectorXd &curvature_ref,
                               const Eigen::VectorXd &velocity_ref,
                               const Eigen::VectorXd &acceleration_ref,
                               double vx_body)
{
    if (!capsule_ || !nlp_config_ || !nlp_dims_ || !nlp_in_ || !nlp_out_ || !nlp_solver_) {
        ROS_ERROR_THROTTLE(1.0, "[MPCInterface::solve] Solver structures NOT initialized!");
        return {0.0, false};
    }

    const double dt = 1.0 / param_.get("odom_frequency");

    std::vector<double> kappa_vec(curvature_ref.data(),
                                  curvature_ref.data() + curvature_ref.size());

    std::vector<double> v_path_vec(velocity_ref.data(),
                                   velocity_ref.data() + velocity_ref.size());

    std::vector<double> a_long_vec(acceleration_ref.data(),
                                   acceleration_ref.data() + acceleration_ref.size());

    // resize to N
    if ((int)kappa_vec.size() != N) {
        ROS_WARN_STREAM("[MPCInterface::solve] Curvature size mismatch! Expected "
                        << N << ", got " << kappa_vec.size() << " -> resizing.");
        if ((int)kappa_vec.size() < N) kappa_vec.resize(N, kappa_vec.empty() ? 0.0 : kappa_vec.back());
        if ((int)kappa_vec.size() > N) kappa_vec.resize(N);
    }

    if ((int)v_path_vec.size() != N) {
        ROS_WARN_STREAM("[MPCInterface::solve] Velocity(v_path) size mismatch! Expected "
                        << N << ", got " << v_path_vec.size() << " -> resizing.");
        const double v_fallback = param_.get("v_target");
        if ((int)v_path_vec.size() < N) v_path_vec.resize(N, v_path_vec.empty() ? v_fallback : v_path_vec.back());
        if ((int)v_path_vec.size() > N) v_path_vec.resize(N);
    }

    // jeśli a_long_ref puste -> fallback z różnic v_path
    if (a_long_vec.empty()) {
        a_long_vec = build_a_long_from_v_path(N, dt, v_path_vec);
    } else if ((int)a_long_vec.size() != N) {
        // dopasuj do N
        if ((int)a_long_vec.size() < N) a_long_vec.resize(N, a_long_vec.empty() ? 0.0 : a_long_vec.back());
        if ((int)a_long_vec.size() > N) a_long_vec.resize(N);
    }

    // vx0_body: preferuj odometrię (x0.vx0_body), fallback na v_path[0]
    double vx0_body = vx_body;
    if (!std::isfinite(vx0_body) || vx0_body <= 0.0) {
        vx0_body = (v_path_vec.empty() ? param_.get("v_target") : v_path_vec.front());
    }

    // build v_vehicle prediction
    std::vector<double> v_vehicle_vec = build_v_vehicle_pred(N, dt, vx0_body, a_long_vec);

    if (!is_initialized_) {
        mpc_ltv_discrete_acados_reset(capsule_, 1);
        reset_initial_guess_splitv_(x0, v_vehicle_vec);
        last_output.assign(N, 0.0);
        is_initialized_ = true;
    }

    double x0_arr[NX];
    x0.to_array(x0_arr);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", x0_arr);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", x0_arr);

    ltv_matrixes_to_acados_splitv_(x0, kappa_vec, v_path_vec, v_vehicle_vec);
    set_cost_to_acados();

    const int status = mpc_ltv_discrete_acados_solve(capsule_);

    if (status == 0) {
        std::vector<double> new_u_traj;
        new_u_traj.reserve(N);

        double u_step[NU];
        for (int i = 0; i < N; ++i) {
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u", u_step);
            new_u_traj.push_back(u_step[0]);
        }

        last_output.clear();
        for (int i = 1; i < N; ++i) last_output.push_back(new_u_traj[i]);
        last_output.push_back(new_u_traj.back());

        const double u_final = new_u_traj[0];

        if (!std::isfinite(u_final)) {
            ROS_ERROR("[MPC DIAG] Solver returned NaN/INF u_final!");
            dump_acados_solver_stats(nlp_solver_, status);
            dump_solution_maxima(nlp_config_, nlp_dims_, nlp_out_, N);
            dump_solution_preview(nlp_config_, nlp_dims_, nlp_out_, N, std::min(10, N));
            is_initialized_ = false;
            last_output.assign(N, 0.0);
            return {0.0, false};
        }

        return {u_final, true};
    }

    ROS_WARN_STREAM("[MPC] Solver FAILED with status: " << status);

    dump_acados_solver_stats(nlp_solver_, status);
    dump_solution_maxima(nlp_config_, nlp_dims_, nlp_out_, N);
    dump_solution_preview(nlp_config_, nlp_dims_, nlp_out_, N, std::min(10, N));

    is_initialized_ = false;
    last_output.assign(N, 0.0);
    return {0.0, false};
}

// ============================================================
// SOLVE — SPLIT-V (fallback: a_long z różnic v_path)
// ============================================================
MPC_Return MPCInterface::solve(const MPC_State &x0,
                               const Eigen::VectorXd &curvature_ref,
                               const Eigen::VectorXd &velocity_ref)
{
    Eigen::VectorXd a_empty; // pusty -> uruchomi fallback w solve(full)
    return solve(x0, curvature_ref, velocity_ref, a_empty,velocity_ref(0));
}

// ============================================================
// Stubs
// ============================================================
void MPCInterface::print_problem_debug(const Eigen::Matrix<double, NX, NX> &,
                                       const Eigen::Matrix<double, NX, NU> &,
                                       const Eigen::Matrix<double, NX, 1>  &,
                                       const MPC_State &)
{
}

void MPCInterface::build_lti_continuous_matrices(Eigen::Matrix<double, NX, NX>&,
                                                 Eigen::Matrix<double, NX, NU>&) const
{
}

void MPCInterface::build_lti_continuous_matrices(Eigen::Matrix<double, NX, NX>&,
                                                 Eigen::Matrix<double, NX, NU>&,
                                                 double) const
{
}

void MPCInterface::push_lti_params_to_acados(double)
{
}

} // namespace v2_control
