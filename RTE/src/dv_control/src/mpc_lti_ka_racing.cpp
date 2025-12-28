#include "mpc_interface.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "acados_solver_mpc_ltv_discrete.h"
#include "acados_c/ocp_nlp_interface.h"

namespace v2_control {

// ============================================================
// Helper: exact discretization via matrix exponential (A,B)->(Ad,Bd)
// (full augmented expm: [A B; 0 0])
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

// =====================================
// KONSTRUKTOR
// =====================================
MPCInterface::MPCInterface()
: capsule_(nullptr),
  nlp_config_(nullptr),
  nlp_dims_(nullptr),
  nlp_in_(nullptr),
  nlp_out_(nullptr),
  is_initialized_(false),
  nlp_solver_(nullptr)
{
    last_output.assign(N, 0.0);
}

MPCInterface::MPCInterface(const ParamBank &P)
: capsule_(nullptr),
  nlp_config_(nullptr),
  nlp_dims_(nullptr),
  nlp_in_(nullptr),
  nlp_out_(nullptr),
  is_initialized_(false),
  nlp_solver_(nullptr)
{
    capsule_ = mpc_ltv_discrete_acados_create_capsule();
    if (!capsule_) {
        ROS_ERROR("[MPC SOLVER] create_capsule returned NULL!");
        return;
    }

    const int st = mpc_ltv_discrete_acados_create(capsule_);
    if (st != 0) {
        ROS_ERROR_STREAM("[MPC SOLVER] Could not create ACADOS solver! Status: " << st);
        return;
    }

    nlp_config_ = mpc_ltv_discrete_acados_get_nlp_config(capsule_);
    nlp_dims_   = mpc_ltv_discrete_acados_get_nlp_dims(capsule_);
    nlp_in_     = mpc_ltv_discrete_acados_get_nlp_in(capsule_);
    nlp_out_    = mpc_ltv_discrete_acados_get_nlp_out(capsule_);
    nlp_solver_ = mpc_ltv_discrete_acados_get_nlp_solver(capsule_);

    param_ = P;
    is_initialized_ = false;
    last_output.assign(N, 0.0);
}

// =====================================
// DESTRUKTOR
// =====================================
MPCInterface::~MPCInterface()
{
    if (capsule_) {
        mpc_ltv_discrete_acados_free(capsule_);
        mpc_ltv_discrete_acados_free_capsule(capsule_);
        capsule_ = nullptr;
    }
}

// =====================================
// RESET INITIAL GUESS (zgodnie z HPP)
// - wpycham x0 w cały horyzont + u=0
// - vref_vec jest tu niewykorzystywany w tej wersji LTI (ale sygnatura ma być zgodna)
// =====================================
void MPCInterface::reset_initial_guess(const MPC_State& x0,
                                       const std::vector<double>& vref_vec)
{
    (void)vref_vec;

    if (!nlp_config_ || !nlp_dims_ || !nlp_in_ || !nlp_out_) return;

    double x_init[NX];
    x0.to_array(x_init);

    double u_init[NU] = {0.0};

    for (int i = 0; i <= N; ++i) {
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", x_init);
        if (i < N) {
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", u_init);
        }
    }
}

// ============================================================
// LTI continuous model around straight driving (small angles, linear tires)
// ============================================================
void MPCInterface::build_lti_continuous_matrices(
    Eigen::Matrix<double, NX, NX>& Ac,
    Eigen::Matrix<double, NX, NU>& Bc,
    double v_ref0) const
{
    const double m  = param_.get("model_m");
    const double Iz = param_.get("model_Iz");
    const double lf = param_.get("model_lf");
    const double lr = param_.get("model_lr");
    const double Cr = param_.get("model_Cr");
    const double Cf = param_.get("model_Cf");

    const double v_eps = 0.5;
    const double v = std::max(v_ref0, v_eps);

    const double omega    = param_.get("model_steer_natural_freq");
    const double damp     = param_.get("model_steer_damping");
    const double omega_sq = omega * omega;

    Ac.setZero();
    Bc.setZero();

    // ey_dot   ≈ vy + v*epsi
    // epsi_dot ≈ r
    Ac(0, 1) = v;
    Ac(0, 2) = 1.0;
    Ac(1, 3) = 1.0;

    // vy_dot
    Ac(2, 2) = -(Cf + Cr) / (m * v);
    Ac(2, 3) = -(Cf * lf - Cr * lr) / (m * v) - v;
    Ac(2, 4) =  (Cf) / m;

    // r_dot
    Ac(3, 2) = -(lf * Cf - lr * Cr) / (Iz * v);
    Ac(3, 3) = -(lf * lf * Cf + lr * lr * Cr) / (Iz * v);
    Ac(3, 4) =  (lf * Cf) / Iz;

    // Servo
    Ac(4, 5) = 1.0;
    Ac(5, 4) = -omega_sq;
    Ac(5, 5) = -2.0 * damp * omega;
    Ac(5, 6) =  omega_sq;

    // d_req_dot = u
    Bc(6, 0) = 1.0;
}

// ============================================================
// Wersja bez v_ref0 (zgodnie z HPP) — fallback na v_target
// ============================================================
void MPCInterface::build_lti_continuous_matrices(
    Eigen::Matrix<double, NX, NX>& Ac,
    Eigen::Matrix<double, NX, NU>& Bc) const
{
    const double v_ref0 = param_.get("v_target");
    build_lti_continuous_matrices(Ac, Bc, v_ref0);
}

// ============================================================
// Push constant LTI (Ad,Bd,Kd=0) into ALL stages (0..N-1)
// ============================================================
void MPCInterface::push_lti_params_to_acados(double v_ref0)
{
    if (!capsule_) return;

    const double dt = 1.0 / param_.get("odom_frequency");

    Eigen::Matrix<double, NX, NX> Ac;
    Eigen::Matrix<double, NX, NU> Bc;
    build_lti_continuous_matrices(Ac, Bc, v_ref0);

    Eigen::Matrix<double, NX, NX> Ad;
    Eigen::Matrix<double, NX, NU> Bd;
    discretize_expm_AB(Ac, Bc, dt, Ad, Bd);

    Eigen::Matrix<double, NX, 1> Kd = Eigen::Matrix<double, NX, 1>::Zero();

    std::vector<double> p_vec;
    p_vec.reserve(NX*NX + NX*NU + NX);

    for (int c = 0; c < NX; ++c)
        for (int r = 0; r < NX; ++r)
            p_vec.push_back(Ad(r, c));

    for (int c = 0; c < NU; ++c)
        for (int r = 0; r < NX; ++r)
            p_vec.push_back(Bd(r, c));

    for (int r = 0; r < NX; ++r)
        p_vec.push_back(Kd(r));

    for (int k = 0; k < N; ++k) {
        mpc_ltv_discrete_acados_update_params(capsule_, k, p_vec.data(), (int)p_vec.size());
    }
}

// =====================================
// USTAWIENIE KOSZTÓW
// =====================================
void MPCInterface::set_cost_to_acados()
{
    if (!nlp_config_ || !nlp_dims_ || !nlp_in_) return;

    const double Q_y      = param_.get("mpc_cost_Q_y");
    const double Q_psi    = param_.get("mpc_cost_Q_psi");
    const double Q_r      = param_.get("mpc_cost_Q_r");
    const double R_ddelta = param_.get("mpc_cost_R_ddelta");
    const double term_scale = 1.0;

    const int ny = NX + NU;
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(ny, ny);
    W(0, 0)   = Q_y;
    W(1, 1)   = Q_psi;
    W(4, 4)   = Q_r;
    W(NX, NX) = R_ddelta;

    const int ny_e = NX;
    Eigen::MatrixXd W_e = Eigen::MatrixXd::Zero(ny_e, ny_e);
    W_e(0, 0) = Q_y   * term_scale;
    W_e(1, 1) = Q_psi * term_scale;

    for (int i = 0; i < N; ++i) {
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "W", W.data());
    }
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N, "W", W_e.data());
}

// =====================================
// SOLVE (3-arg) — LTI straight, ignore curvature_ref
// =====================================
MPC_Return MPCInterface::solve(const MPC_State &x0,
                               const Eigen::VectorXd &/*curvature_ref*/,
                               const Eigen::VectorXd &velocity_ref)
{
    if (!capsule_ || !nlp_config_ || !nlp_dims_ || !nlp_in_ || !nlp_out_) {
        ROS_ERROR_THROTTLE(1.0, "[MPCInterface::solve] Solver structures NOT initialized!");
        return {0.0, false};
    }

    double v_ref0 = param_.get("v_target");
    if (velocity_ref.size() > 0 && std::isfinite(velocity_ref(0))) {
        v_ref0 = velocity_ref(0);
    }

    // 1) load constant LTI params
    push_lti_params_to_acados(v_ref0);

    // 2) cold start
    if (!is_initialized_) {
        mpc_ltv_discrete_acados_reset(capsule_, 1);

        // zgodnie z HPP: reset_initial_guess chce vector
        std::vector<double> vref_vec_dummy(1, v_ref0);
        reset_initial_guess(x0, vref_vec_dummy);

        last_output.assign(N, 0.0);
        is_initialized_ = true;
    }

    // 3) fix x0 via bounds at stage 0
    double x0_arr[NX];
    x0.to_array(x0_arr);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", x0_arr);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", x0_arr);

    // 4) costs
    set_cost_to_acados();

    // 5) solve
    const int status = mpc_ltv_discrete_acados_solve(capsule_);

    if (status == 0) {
        double u0[NU];
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u0);

        std::vector<double> new_u_traj;
        new_u_traj.reserve(N);
        for (int i = 0; i < N; ++i) {
            double ut[NU];
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u", ut);
            new_u_traj.push_back(ut[0]);
        }

        if (!new_u_traj.empty()) {
            last_output.assign(new_u_traj.begin() + 1, new_u_traj.end());
            last_output.push_back(new_u_traj.back());
        }

        if (!std::isfinite(u0[0])) {
            ROS_WARN("[MPC] u0 is NaN/Inf -> reset");
            is_initialized_ = false;
            last_output.assign(N, 0.0);
            return {0.0, false};
        }

        return {u0[0], true};
    }

    ROS_WARN("[MPC] Fail status: %d", status);
    is_initialized_ = false;
    last_output.assign(N, 0.0);
    return {0.0, false};
}

// =====================================
// SOLVE (5-arg) — zgodny z HPP, ale w tej wersji LTI ignoruję extra args
// (żeby build nie wywalał errorów na brak definicji/sygnatury)
// =====================================
MPC_Return MPCInterface::solve(const MPC_State &x0,
                               const Eigen::VectorXd &curvature_ref,
                               const Eigen::VectorXd &velocity_ref,
                               const Eigen::VectorXd &acceleration_ref,
                               double vx0_body)
{
    (void)acceleration_ref;
    (void)vx0_body;
    return solve(x0, curvature_ref, velocity_ref);
}

} // namespace v2_control
