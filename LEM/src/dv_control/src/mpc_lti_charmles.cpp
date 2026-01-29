#include "mpc_interface.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <limits>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <unsupported/Eigen/MatrixFunctions> // .exp()

// ACADOS C interface
#include "acados_solver_mpc_ltv_discrete.h"
#include "acados_c/ocp_nlp_interface.h"
namespace v2_control {

// ============================================================
// ===================== DIAGNOSTYKA ==========================
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

        if (k < N_hor) {
            oss << " u=[";
            for (int i = 0; i < NU; ++i) {
                oss << uk[i];
                if (i + 1 < NU) oss << ", ";
            }
            oss << "]";
        }

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

    double max_abs_ddelta = 0.0;

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
            max_abs_ddelta = std::max(max_abs_ddelta, std::abs(uk[0]));
        }
    }

    ROS_WARN_STREAM(std::fixed << std::setprecision(6)
        << "[MPC DIAG] maxima over horizon:"
        << " max|ddelta|=" << max_abs_ddelta
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
// Discretization helpers (exact)
// ============================================================

// augment expm: [A B; 0 0] -> Ad,Bd (dokładnie, bez A^{-1})
static inline void discretize_expm_AB(
    const Eigen::Matrix<double, NX, NX>& A,
    const Eigen::Matrix<double, NX, NU>& B,
    double dt,
    Eigen::Matrix<double, NX, NX>& Ad,
    Eigen::Matrix<double, NX, NU>& Bd)
{
    Eigen::Matrix<double, NX + NU, NX + NU> M =
        Eigen::Matrix<double, NX + NU, NX + NU>::Zero();

    M.block<NX, NX>(0, 0)  = A;
    M.block<NX, NU>(0, NX) = B;

    Eigen::Matrix<double, NX + NU, NX + NU> E = (M * dt).exp();
    Ad = E.block<NX, NX>(0, 0);
    Bd = E.block<NX, NU>(0, NX);
}

// ============================================================
// Nonlinear continuous dynamics (do RK4 -> Kd)
// u = [ddelta]
// ============================================================

static inline Eigen::Matrix<double, NX, 1> f_consistent_continuous(
    const Eigen::Matrix<double, NX, 1>& x,
    const Eigen::Matrix<double, NU, 1>& u,
    double kappa,
    double v_ref0,
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

    // x = [ey, epsi, vy, r, delta, d_delta, d_req]
    const double epsi    = x(1);
    const double vy      = x(2);
    const double r       = x(3);
    const double delta   = x(4);
    const double d_delta = x(5);
    const double d_req   = x(6);

    const double s_epsi  = std::sin(epsi);
    const double c_epsi  = std::cos(epsi);
    const double c_delta = std::cos(delta);

    const double v = v_ref0;

    const double yf = vy + lf * r;
    const double yr = vy - lr * r;

    const double alpha_f = delta - std::atan2(yf, v);
    const double alpha_r = -std::atan2(yr, v);

    const double Fyf = Cf * alpha_f;
    const double Fyr = Cr * alpha_r;

    const double ddelta = u(0);
    const double mtv    = (NU > 1) ? u(1) : 0.0;

    Eigen::Matrix<double, NX, 1> xdot;
    xdot.setZero();

    xdot(0) = vy * c_epsi + v * s_epsi;                  // ey_dot
    xdot(1) = r - kappa * v;                             // epsi_dot
    xdot(2) = (Fyf * c_delta + Fyr) / m - v * r;         // vy_dot
    xdot(3) = (lf * Fyf * c_delta - lr * Fyr) / Iz + (mtv / Iz); // r_dot + tv

    // Servo
    xdot(4) = d_delta;                                                     // delta_dot
    xdot(5) = -omega_sq * delta - 2.0 * damp * omega * d_delta + omega_sq * d_req; // d_delta_dot
    xdot(6) = ddelta;                                                      // d_req_dot

    return xdot;
}

static inline Eigen::Matrix<double, NX, 1> rk4_step_consistent(
    const Eigen::Matrix<double, NX, 1>& x,
    const Eigen::Matrix<double, NU, 1>& u,
    double kappa,
    double v_ref0,
    double dt,
    const ParamBank& param)
{
    const auto k1 = f_consistent_continuous(x,                 u, kappa, v_ref0, param);
    const auto k2 = f_consistent_continuous(x + 0.5*dt*k1,     u, kappa, v_ref0, param);
    const auto k3 = f_consistent_continuous(x + 0.5*dt*k2,     u, kappa, v_ref0, param);
    const auto k4 = f_consistent_continuous(x + dt*k3,         u, kappa, v_ref0, param);

    return x + (dt/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

// =====================================
// CTORS / DTOR
// =====================================

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

    last_output.assign(N, Eigen::Matrix<double, NU, 1>::Zero());
}

MPCInterface::MPCInterface(const ParamBank &P)
{
    ROS_INFO("[MPCInterface] Constructing MPCInterface with ParamBank");

    capsule_ = mpc_ltv_discrete_acados_create_capsule();
    if (!capsule_) {
        ROS_ERROR("[MPC SOLVER] create_capsule returned NULL!");
        nlp_config_ = nullptr;
        nlp_dims_   = nullptr;
        nlp_in_     = nullptr;
        nlp_out_    = nullptr;
        nlp_solver_ = nullptr;
        is_initialized_ = false;
        last_output.assign(N, Eigen::Matrix<double, NU, 1>::Zero());
        return;
    }

    const int status = mpc_ltv_discrete_acados_create(capsule_);
    if (status != 0) {
        ROS_ERROR_STREAM("[MPC SOLVER] Could not create ACADOS solver! Status: " << status);

        nlp_config_ = nullptr;
        nlp_dims_   = nullptr;
        nlp_in_     = nullptr;
        nlp_out_    = nullptr;
        nlp_solver_ = nullptr;

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
    last_output.assign(N, Eigen::Matrix<double, NU, 1>::Zero());
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

// =====================================
// RESET INITIAL GUESS (używa v_ref0)
// =====================================
void MPCInterface::reset_initial_guess(const MPC_State& x0, double v_ref0)
{
    ROS_INFO("[MPCInterface] Resetting initial guess (Straight Line Prediction)");
    if (!nlp_config_ || !nlp_dims_ || !nlp_in_ || !nlp_out_) return;

    const double dt = 1.0 / param_.get("odom_frequency");

    const double current_ey   = x0.ey;
    const double current_epsi = x0.epsi;

    double x_traj[NX];
    double u_zero[NU] = {0.0};

    for (int i = 0; i <= N; ++i) {
        const double t = i * dt;
        const double predicted_ey = current_ey + (v_ref0 * std::sin(current_epsi)) * t;

        x_traj[0] = predicted_ey;
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

// ======================================================================
// Continuous Jacobian
// Bc(6,0)=1, Bc(3,1)=1/Iz  (MTV)
// ======================================================================
void MPCInterface::calculate_continuous_jacobian(
    const Eigen::Matrix<double, NX, 1>& x,
    double v_ref0,
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

    const double v = v_ref0;

    // x = [ey, epsi, vy, r, delta, d_delta, d_req]
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

    const double alpha_f = delta - std::atan2(yf, v);
    const double alpha_r = -std::atan2(yr, v);

    const double Fyf = Cf * alpha_f;
    const double Fyr = Cr * alpha_r;

    const double denom_f = v*v + yf*yf;
    const double denom_r = v*v + yr*yr;

    const double datan_f_dyf = (denom_f > 1e-12) ? (v / denom_f) : 0.0;
    const double datan_r_dyr = (denom_r > 1e-12) ? (v / denom_r) : 0.0;

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

    // ey_dot = vy*cos(epsi) + v*sin(epsi)
    Ac(0, 1) = -vy * s_epsi + v * c_epsi;
    Ac(0, 2) =  c_epsi;

    // epsi_dot = r - v*kappa  -> po x: tylko Ac(1,3)=1
    Ac(1, 3) = 1.0;

    // vy_dot = (Fyf*cos(delta)+Fyr)/m - v*r
    Ac(2, 2) = (1.0 / m) * (dFyf_d_vy * c_delta + dFyr_d_vy);
    Ac(2, 3) = (1.0 / m) * (dFyf_d_r  * c_delta + dFyr_d_r) - v;

    const double d_vy_term_d_delta = dFyf_d_delta * c_delta - Fyf * s_delta;
    Ac(2, 4) = (1.0 / m) * d_vy_term_d_delta;

    // r_dot = (lf*Fyf*cos(delta) - lr*Fyr)/Iz + mtv/Iz
    Ac(3, 2) = (1.0 / Iz) * (lf * dFyf_d_vy * c_delta - lr * dFyr_d_vy);
    Ac(3, 3) = (1.0 / Iz) * (lf * dFyf_d_r  * c_delta - lr * dFyr_d_r);

    const double d_r_term_d_delta = lf * (dFyf_d_delta * c_delta - Fyf * s_delta);
    Ac(3, 4) = (1.0 / Iz) * d_r_term_d_delta;

    // Servo
    Ac(4, 5) = 1.0;
    Ac(5, 4) = -omega_sq;
    Ac(5, 5) = -2.0 * damp * omega;
    Ac(5, 6) =  omega_sq;

    // inputs
    Bc(6, 0) = 1.0;               // d_req_dot = ddelta
}

// ======================================================================
// FROZEN dynamics push to ACADOS:
// - wybieram punkt (x0,u0,kappa0,v0)
// - liczę Ac,Bc tylko raz
// - dyskretyzuję do Ad,Bd
// - liczę Kd tak, żeby model pasował do nieliniowego kroku RK4
// - pushuję IDENTYCZNIE to samo do wszystkich stage k=0..N-1
// ======================================================================
void MPCInterface::ltv_matrixes_to_acados(const MPC_State& x0,
                                         const std::vector<double>& kappa_vec,
                                         const std::vector<double>& vref_vec)
{
    if (!capsule_) return;

    const double dt = 1.0 / param_.get("odom_frequency");

    // v0 = vref[0]
    double v0 = param_.get("v_target");
    if (!vref_vec.empty() && std::isfinite(vref_vec[0])) v0 = vref_vec[0];

    // kappa0 = curvature_ref[0] (freezing w aktualnym punkcie)
    double kappa0 = 0.0;
    if (!kappa_vec.empty() && std::isfinite(kappa_vec[0])) kappa0 = kappa_vec[0];

    // x_lin
    Eigen::Matrix<double, NX, 1> x_lin;
    x_lin << x0.ey, x0.epsi, x0.vy, x0.r, x0.delta, x0.d_delta, x0.delta_request;

    // u0 = last_output[0]
    Eigen::Matrix<double, NU, 1> u0 = Eigen::Matrix<double, NU, 1>::Zero();
    if (!last_output.empty()) u0 = last_output[0];

    // Jacobian raz
    Eigen::Matrix<double, NX, NX> Ac;
    Eigen::Matrix<double, NX, NU> Bc;
    calculate_continuous_jacobian(x_lin, v0, Ac, Bc);

    // dyskretyzacja expm (dokładna)
    Eigen::Matrix<double, NX, NX> Ad;
    Eigen::Matrix<double, NX, NU> Bd;
    discretize_expm_AB(Ac, Bc, dt, Ad, Bd);

    // Kd: dokładnie jak w teorii:
    // x1_true = RK4(x0,u0,kappa0,v0)
    // Kd = x1_true - (Ad*x0 + Bd*u0)
    const Eigen::Matrix<double, NX, 1> x1_true =
        rk4_step_consistent(x_lin, u0, kappa0, v0, dt, param_);

    Eigen::Matrix<double, NX, 1> Kd =
        x1_true - (Ad * x_lin + Bd * u0);

    // sanity-check
    {
        Eigen::MatrixXd Ad_d = Ad;
        Eigen::MatrixXd Bd_d = Bd;
        Eigen::VectorXd Kd_d = Kd;

        if (!eigen_all_finite(Ad_d) || !eigen_all_finite(Bd_d) || !eigen_all_finite_vec(Kd_d)) {
            ROS_ERROR_STREAM("[MPC DIAG] NaN/INF in FROZEN params"
                << " | v0=" << v0
                << " | kappa0=" << kappa0
                << " | x0=" << x_lin.transpose()
                << " | u0=" << u0.transpose());

            ROS_ERROR_STREAM("[MPC DIAG] norms: ||Ad||=" << Ad.norm()
                << " ||Bd||=" << Bd.norm()
                << " ||Kd||=" << Kd.norm());
        }
    }

    // przygotuj p_vec RAZ i użyj do wszystkich stage
    std::vector<double> p_vec;
    p_vec.reserve(NX*NX + NX*NU + NX);

    // Ad (column-major jak w twoim kodzie)
    for (int c = 0; c < NX; ++c)
        for (int r = 0; r < NX; ++r)
            p_vec.push_back(Ad(r, c));

    // Bd
    for (int c = 0; c < NU; ++c)
        for (int r = 0; r < NX; ++r)
            p_vec.push_back(Bd(r, c));

    // Kd
    for (int r = 0; r < NX; ++r)
        p_vec.push_back(Kd(r));

    // push identycznych parametrów do wszystkich stage
    for (int k = 0; k < N; ++k) {
        mpc_ltv_discrete_acados_update_params(capsule_, k, p_vec.data(), (int)p_vec.size());
    }
}

// =====================================
// Koszty (NU=2 -> dodaj wagę na mtv)
// =====================================
void MPCInterface::set_cost_to_acados()
{
    if (!nlp_config_ || !nlp_dims_ || !nlp_in_) return;

    const double Q_y      = param_.get("mpc_cost_Q_y");
    const double Q_psi    = param_.get("mpc_cost_Q_psi");
    const double R_ddelta = param_.get("mpc_cost_R_ddelta");
    const double Q_r      = param_.get("mpc_cost_Q_r");
    const double Q_delta  = param_.get("mpc_cost_Q_delta");

    // jeśli nie masz w ParamBanku -> daj małe, ale nie 0

    const int ny = NX + NU;
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(ny, ny);

    // stany (kolejność: ey epsi vy r delta d_delta d_req)
    W(0, 0) = Q_y;
    W(1, 1) = Q_psi;
    W(3, 3) = Q_r;
    W(4, 4) = Q_delta;

    // wejścia
    W(NX + 0, NX + 0) = R_ddelta;

    const int ny_e = NX;
    Eigen::MatrixXd W_e = Eigen::MatrixXd::Zero(ny_e, ny_e);
    W_e(0, 0) = Q_y;
    W_e(1, 1) = Q_psi;

    for (int i = 0; i < N; ++i)
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "W", W.data());

    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N, "W", W_e.data());
}

// =====================================
// SOLVE
// return = {ddelta_opt, mtv_opt, success, next_yaw_rate}
// =====================================
MPC_Return MPCInterface::solve(const MPC_State &x0,
                               const Eigen::VectorXd &curvature_ref,
                               const Eigen::VectorXd &velocity_ref)
{
    if (!capsule_ || !nlp_config_ || !nlp_dims_ || !nlp_in_ || !nlp_out_ || !nlp_solver_) {
        ROS_ERROR_THROTTLE(1.0, "[MPCInterface::solve] Solver structures NOT initialized!");
        return {0.0, 0.0, false, x0.r};
    }

    // kappa_vec (N)
    std::vector<double> kappa_vec(curvature_ref.data(),
                                  curvature_ref.data() + curvature_ref.size());
    if ((int)kappa_vec.size() != N) {
        if ((int)kappa_vec.size() < N) kappa_vec.resize(N, kappa_vec.empty() ? 0.0 : kappa_vec.back());
        if ((int)kappa_vec.size() > N) kappa_vec.resize(N);
    }

    // vref_vec (nie musi mieć N, bierzemy vref[0])
    std::vector<double> vref_vec(velocity_ref.data(),
                                 velocity_ref.data() + velocity_ref.size());

    double v0 = param_.get("v_target");
    if (!vref_vec.empty() && std::isfinite(vref_vec[0])) v0 = vref_vec[0];

    // init
    if (!is_initialized_) {
        mpc_ltv_discrete_acados_reset(capsule_, 1);
        reset_initial_guess(x0, v0);

        last_output.assign(N, Eigen::Matrix<double, NU, 1>::Zero());
        is_initialized_ = true;
    }

    // constraints x0
    double x0_arr[NX];
    x0.to_array(x0_arr);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", x0_arr);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", x0_arr);

    // frozen params push
    ltv_matrixes_to_acados(x0, kappa_vec, vref_vec);
    set_cost_to_acados();

    const int status = mpc_ltv_discrete_acados_solve(capsule_);

    if (status == 0)
    {
        // read full u traj
        std::vector<Eigen::Matrix<double, NU, 1>> new_u_traj;
        new_u_traj.reserve(N);

        double u_step[NU];
        for (int i = 0; i < N; ++i) {
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u", u_step);

            Eigen::Matrix<double, NU, 1> ui = Eigen::Matrix<double, NU, 1>::Zero();
            ui(0) = u_step[0];

            new_u_traj.push_back(ui);
        }

        // shift to last_output
        if (!new_u_traj.empty()) {
            last_output.clear();
            last_output.reserve(N);

            for (int i = 1; i < N; ++i) last_output.push_back(new_u_traj[i]);
            last_output.push_back(new_u_traj.back());
        }

        // u0
        const double ddelta_opt = new_u_traj.empty() ? 0.0 : new_u_traj[0](0);

        if (!std::isfinite(ddelta_opt)) {
            ROS_ERROR("[MPC DIAG] Solver returned NaN/INF in u0!");
            dump_acados_solver_stats(nlp_solver_, status);
            dump_solution_maxima(nlp_config_, nlp_dims_, nlp_out_, N);
            dump_solution_preview(nlp_config_, nlp_dims_, nlp_out_, N, std::min(10, N));

            is_initialized_ = false;
            last_output.assign(N, Eigen::Matrix<double, NU, 1>::Zero());
            return {0.0, 0.0, false, x0.r};
        }

        // next predicted yaw rate r from x(k=1)
        double x1[NX];
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 1, "x", x1);

        double next_r = x1[3];
        if (!std::isfinite(next_r)) next_r = x0.r;

        return {ddelta_opt, 0.0, true, next_r};
    }

    // FAIL
    ROS_WARN_STREAM("[MPC] Solver FAILED with status: " << status);
    dump_acados_solver_stats(nlp_solver_, status);
    dump_solution_maxima(nlp_config_, nlp_dims_, nlp_out_, N);
    dump_solution_preview(nlp_config_, nlp_dims_, nlp_out_, N, std::min(10, N));

    is_initialized_ = false;
    last_output.assign(N, Eigen::Matrix<double, NU, 1>::Zero());
    return {0.0, 0.0, false, x0.r};
}

MPC_Return MPCInterface::solve(const MPC_State &x0,
                               const Eigen::VectorXd &curvature_ref,
                               const Eigen::VectorXd &velocity_ref,
                               const Eigen::VectorXd &acceleration_ref,
                               double vx0_body)
{
    (void)acceleration_ref;
    (void)vx0_body;

    // Fallback: użyj prostszej wersji solve() (bez split-v).
    // To naprawia linkowanie; docelowo można tu zaimplementować pełny split-v.
    return this->solve(x0, curvature_ref, velocity_ref);
}

// ============================================================
// Stuby / legacy
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

} // namespace v2_control
