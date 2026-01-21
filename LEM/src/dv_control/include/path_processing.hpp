#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

#include "spline.hpp"     // TrackSpline2D
#include "ParamBank.hpp"
#include "Vec2.hpp"
#include <ros/ros.h>
#include "mpc_interface.hpp"

namespace v2_control {

// =====================================================
// Basic structs
// =====================================================
struct PathProcessResult
{
    int N = 0;
    Eigen::VectorXd X_ref;
    Eigen::VectorXd Y_ref;
    Eigen::VectorXd curvature;

    bool valid = false;
    bool curv_eligible = false;
    bool geo_eligible = false;
    bool mpc_eligible = false;
    bool all_path_eligible = false;

    Eigen::VectorXd acceleration_ref;
    Eigen::VectorXd velocity_ref;

    double yaw0 = 0.0;
};

// =====================================================
// Helpers
// =====================================================
static inline double yawFrom2Pts(double x0, double y0, double x1, double y1)
{
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double n2 = dx*dx + dy*dy;
    if (n2 < 1e-12) return 0.0;
    return std::atan2(dy, dx);
}



inline Vec2 projectPointOnSegment(const Vec2& A, const Vec2& B, const Vec2& P)
{
    Vec2 AB = B - A;
    const double len2 = AB.x * AB.x + AB.y * AB.y;

    if (len2 < 1e-12) return A;

    double t = ((P.x - A.x) * AB.x + (P.y - A.y) * AB.y) / len2;
    t = std::clamp(t, 0.0, 1.0);

    return Vec2(A.x + t * AB.x, A.y + t * AB.y);
}

inline PathProcessResult makeGeoInvalidResult(const Eigen::VectorXd& X,
                                             const Eigen::VectorXd& Y)
{
    PathProcessResult r;
    const int N = static_cast<int>(X.size());
    r.N = N;
    r.X_ref = X;
    r.Y_ref = Y;
    r.curvature = Eigen::VectorXd::Zero(N);
    r.velocity_ref = Eigen::VectorXd::Zero(N);
    r.acceleration_ref = Eigen::VectorXd::Zero(N);
    r.valid = true;
    r.geo_eligible = false;
    r.curv_eligible = false;
    r.mpc_eligible = false;
    r.all_path_eligible = false;

    if (N >= 2) r.yaw0 = yawFrom2Pts(r.X_ref(0), r.Y_ref(0), r.X_ref(1), r.Y_ref(1));
    else        r.yaw0 = 0.0;

    return r;
}

inline PathProcessResult makeCurvInvalidResult(const Eigen::VectorXd& X,
                                              const Eigen::VectorXd& Y)
{
    PathProcessResult r;
    const int N = static_cast<int>(X.size());
    r.N = N;
    r.X_ref = X;
    r.Y_ref = Y;
    r.curvature = Eigen::VectorXd::Zero(N);
    r.velocity_ref = Eigen::VectorXd::Zero(N);
    r.acceleration_ref = Eigen::VectorXd::Zero(N);
    r.valid = true;
    r.geo_eligible = true;
    r.curv_eligible = false;
    r.mpc_eligible = false;
    r.all_path_eligible = false;

    if (N >= 2) r.yaw0 = yawFrom2Pts(r.X_ref(0), r.Y_ref(0), r.X_ref(1), r.Y_ref(1));
    else        r.yaw0 = 0.0;

    return r;
}

inline std::vector<Vec2> buildPathBase(const Eigen::VectorXd& X,
                                       const Eigen::VectorXd& Y)
{
    const int N = static_cast<int>(X.size());
    std::vector<Vec2> path;
    path.reserve(N);
    for (int i = 0; i < N; ++i) path.emplace_back(X(i), Y(i));
    return path;
}

inline Eigen::VectorXd computeCurvature(const std::vector<Vec2>& path)
{
    const int K = static_cast<int>(path.size());
    Eigen::VectorXd kappa = Eigen::VectorXd::Zero(K);

    if (K < 3) return kappa;

    for (int i = 1; i < K - 1; ++i) {
        const double x0 = path[i - 1].x;
        const double y0 = path[i - 1].y;
        const double x1 = path[i].x;
        const double y1 = path[i].y;
        const double x2 = path[i + 1].x;
        const double y2 = path[i + 1].y;

        const double vx1 = x1 - x0;
        const double vy1 = y1 - y0;
        const double vx2 = x2 - x1;
        const double vy2 = y2 - y1;

        const double a = std::hypot(vx1, vy1);
        const double b = std::hypot(vx2, vy2);
        const double c = std::hypot(x2 - x0, y2 - y0);

        const double cross = vx1 * vy2 - vy1 * vx2;
        const double denom = a * b * c;

        kappa(i) = (denom > 1e-9) ? 2.0 * cross / denom : 0.0;
    }

    kappa(0)     = kappa(1);
    kappa(K - 1) = kappa(K - 2);
    return kappa;
}

inline std::vector<Vec2> sampleSplineByS(const TrackSpline2D& sp,
                                         double ds,
                                         size_t max_points = 0)
{
    std::vector<Vec2> out;
    if (!sp.valid() || ds <= 1e-9) return out;

    const double L = sp.totalLength();
    if (L <= 1e-9) return out;

    size_t N = (size_t)std::floor(L / ds) + 1;
    if (max_points > 0) N = std::min(N, max_points);

    out.reserve(N + 2);

    for (size_t i = 0; i < N; ++i) {
        const double s = std::min(L, (double)i * ds);
        out.push_back(sp.eval(s));
    }

    if (!sp.isClosed()) {
        const Vec2 endp = sp.eval(L);
        if (out.empty() ||
            std::hypot(out.back().x - endp.x, out.back().y - endp.y) > 1e-6)
        {
            if (max_points == 0 || out.size() < max_points) out.push_back(endp);
        }
    }
    return out;
}

// =====================================================
// Projection on polyline (existing path-process logic)
// =====================================================
struct ProjectionInfo {
    int index = 0;
    Vec2 point;
};

inline ProjectionInfo findProjectionOnPath(const std::vector<Vec2>& path,
                                           const Vec2& Pbol,
                                           int maxSegmentsToSearch)
{
    ProjectionInfo info;

    if (path.size() < 2) {
        info.index = 0;
        info.point = (path.empty() ? Pbol : path.front());
        return info;
    }

    // if behind first segment start -> return projection on infinite extension (as you had)
    {
        const Vec2 A0 = path[0];
        const Vec2 B0 = path[1];
        const Vec2 AB0 = B0 - A0;
        const double len2_0 = AB0.x * AB0.x + AB0.y * AB0.y;

        if (len2_0 >= 1e-6) {
            const double t0 = ((Pbol.x - A0.x) * AB0.x + (Pbol.y - A0.y) * AB0.y) / len2_0;
            if (t0 < 0.0) {
                info.point = Vec2(A0.x + t0 * AB0.x, A0.y + t0 * AB0.y);
                info.index = 0;
                return info;
            }
        }
    }

    const int N = static_cast<int>(path.size());
    const int search_limit = std::min(N - 1, maxSegmentsToSearch);

    double best_d2 = 1e30;
    for (int i = 1; i <= search_limit; ++i) {
        const Vec2 A = path[i - 1];
        const Vec2 B = path[i];
        const Vec2 Pp = projectPointOnSegment(A, B, Pbol);

        const double dx = Pp.x - Pbol.x;
        const double dy = Pp.y - Pbol.y;
        const double d2 = dx * dx + dy * dy;

        if (d2 < best_d2) {
            best_d2 = d2;
            info.point = Pp;
            info.index = i;
        }
    }

    return info;
}

inline double computeRemainingPathLength(const std::vector<Vec2>& path,
                                         int projIndex,
                                         const Vec2& projPoint)
{
    const int N = static_cast<int>(path.size());
    if (N == 0) return 0.0;

    double dist = 0.0;

    if (projIndex > 0 && projIndex < N) {
        dist += std::hypot(path[projIndex].x - projPoint.x,
                           path[projIndex].y - projPoint.y);
    }

    for (int i = projIndex; i < N - 1; ++i) {
        const double dx = path[i + 1].x - path[i].x;
        const double dy = path[i + 1].y - path[i].y;
        dist += std::hypot(dx, dy);
    }
    return dist;
}

inline double guessSFromProjection(const std::vector<Vec2>& path,
                                   const ProjectionInfo& proj)
{
    if (path.size() < 2) return 0.0;

    const int i = std::clamp(proj.index, 1, (int)path.size() - 1);

    double s = 0.0;
    for (int k = 0; k < i - 1; ++k) {
        const double dx = path[k + 1].x - path[k].x;
        const double dy = path[k + 1].y - path[k].y;
        s += std::hypot(dx, dy);
    }

    const double dxp = proj.point.x - path[i - 1].x;
    const double dyp = proj.point.y - path[i - 1].y;
    s += std::hypot(dxp, dyp);

    return s;
}

// =====================================================
// MPC-eligible path building (existing logic)
// =====================================================
inline PathProcessResult processMpcEligiblePath_bolide_behind_path(
    const TrackSpline2D& spline,
    const ProjectionInfo projection,
    const ParamBank& P)
{
    PathProcessResult result;

    const int K = static_cast<int>(P.get("mpc_N"));
    result.N = K;
    result.X_ref.resize(K);
    result.Y_ref.resize(K);
    result.curvature.resize(K);
    result.velocity_ref = Eigen::VectorXd::Constant(K, P.get("v_target"));
    result.acceleration_ref = Eigen::VectorXd::Zero(K);
    result.valid = true;
    result.geo_eligible = true;
    result.curv_eligible = true;
    result.mpc_eligible = true;
    result.all_path_eligible = false;

    Eigen::VectorXd curv_temp_ref(K);
    Eigen::VectorXd X_temp(K);
    Eigen::VectorXd Y_temp(K);

    const double dx0 = spline.getX(0.0) - projection.point.x;
    const double dy0 = spline.getY(0.0) - projection.point.y;

    double dist_to_first_path_point = std::hypot(dx0, dy0);
    dist_to_first_path_point = std::max(dist_to_first_path_point, 1e-9);

    int points_to_add = static_cast<int>(std::ceil(dist_to_first_path_point / P.get("mpc_spatial_step")));
    points_to_add = std::max(1, points_to_add - 1);

    const double eps = 1e-6;
    if (points_to_add == 1) {
        points_to_add = 1;
    } else if (std::fabs(dist_to_first_path_point - 2 * P.get("mpc_spatial_step")) < eps) {
        points_to_add = 2;
    } else {
        points_to_add += 1;
    }

    points_to_add = std::min(points_to_add, K - 1);

    for (int i = 0; i < points_to_add; i++) {
        X_temp(i) = projection.point.x + i * P.get("mpc_spatial_step") / dist_to_first_path_point * dx0;
        Y_temp(i) = projection.point.y + i * P.get("mpc_spatial_step") / dist_to_first_path_point * dy0;
        curv_temp_ref(i) = 0.0;
    }

    const double dist_from_last_added_point_to_spline_start =
        -dist_to_first_path_point + (points_to_add - 1) * P.get("mpc_spatial_step");

    double s = std::max(dist_from_last_added_point_to_spline_start + P.get("mpc_spatial_step"), 0.0);

    for (int i = points_to_add; i < K; i++) {
        X_temp(i) = spline.getX(s);
        Y_temp(i) = spline.getY(s);
        curv_temp_ref(i) = spline.getCurvature(s);

        s += P.get("mpc_spatial_step");
        if (!spline.isClosed() && s > spline.totalLength()) s = spline.totalLength();
    }

    result.X_ref = X_temp;
    result.Y_ref = Y_temp;
    result.curvature = curv_temp_ref;

    if (K >= 2) result.yaw0 = yawFrom2Pts(result.X_ref(0), result.Y_ref(0), result.X_ref(1), result.Y_ref(1));
    else        result.yaw0 = 0.0;

    return result;
}

inline PathProcessResult processMpcEligiblePath_bolide_on_path(
    const TrackSpline2D& spline,
    const std::vector<Vec2>& path_dense_for_guess,
    const ProjectionInfo projection,
    const ParamBank& P)
{
    PathProcessResult result;

    const int K = static_cast<int>(P.get("mpc_N"));
    result.N = K;
    result.X_ref.resize(K);
    result.Y_ref.resize(K);
    result.curvature.resize(K);
    result.velocity_ref = Eigen::VectorXd::Constant(K, P.get("v_target"));
    result.acceleration_ref = Eigen::VectorXd::Zero(K);
    result.valid = true;
    result.geo_eligible = true;
    result.curv_eligible = true;
    result.mpc_eligible = true;
    result.all_path_eligible = false;

    result.X_ref(0) = projection.point.x;
    result.Y_ref(0) = projection.point.y;

    const double s_guess = guessSFromProjection(path_dense_for_guess, projection);
    double s = spline.projectToS(projection.point, s_guess, 2.0, 41, 6);

    result.yaw0 = spline.getYaw(s);
    result.curvature(0) = spline.getCurvature(s);

    for (int i = 1; i < K; ++i) {
        s += P.get("mpc_spatial_step");
        if (!spline.isClosed() && s > spline.totalLength()) s = spline.totalLength();

        result.X_ref(i) = spline.getX(s);
        result.Y_ref(i) = spline.getY(s);
        result.curvature(i) = spline.getCurvature(s);
    }

    return result;
}

// =====================================================
// Velocity planner (NEW fixed implementation)
// =====================================================
struct SpeedProfileGeom
{
    double s0 = 0.0;
    double ds = 0.5;
    double S_plan = 0.0;
    std::vector<double> v;
    std::vector<double> kappa;
    std::vector<double> a;
};

static inline double safeSqrt(double x) { return std::sqrt(std::max(0.0, x)); }

static inline double wrapS(double s, double L)
{
    if (L <= 1e-12) return 0.0;
    s = std::fmod(s, L);
    if (s < 0.0) s += L;
    return s;
}

// friction ellipse longitudinal availability (kept)
static inline double longAvail(double v, double kappa, double a_long_max, double a_lat_max)
{
    const double ay = v*v * std::abs(kappa);
    const double r  = std::min(1.0, ay / std::max(1e-9, a_lat_max));
    const double term = std::max(0.0, 1.0 - r*r);
    return a_long_max * std::sqrt(term);
}

static inline void accelBoundsAt(
    double v_here, double k_here,
    double a_acc_max, double a_dec_max, double a_lat_max,
    double& a_min_out, double& a_max_out)
{
    a_max_out =  longAvail(v_here, k_here, a_acc_max, a_lat_max);   // >=0
    a_min_out = -longAvail(v_here, k_here, a_dec_max, a_lat_max);   // <=0
}

static inline std::vector<double> buildVSat(
    const std::vector<double>& kappa,
    double v_max,
    double a_lat_max)
{
    const int N = (int)kappa.size();
    std::vector<double> vsat(N, v_max);
    for (int i = 0; i < N; ++i) {
        const double kk = std::abs(kappa[i]);
        double v_kappa = v_max;
        if (kk > 1e-9) v_kappa = std::sqrt(std::max(0.0, a_lat_max / kk));
        vsat[i] = std::clamp(v_kappa, 0.0, v_max);
    }
    return vsat;
}

// forward: your 3 cases
static inline void forwardPass_threeCases(
    const std::vector<double>& kappa,
    const std::vector<double>& vsat,
    double ds,
    double v_min,
    double v_max,
    double a_acc_max,
    double a_dec_max,
    double a_lat_max,
    double v0,
    std::vector<double>& v,
    std::vector<double>& a,
    std::vector<bool>& is_valid)
{
    const int N = (int)kappa.size();
    v.assign(N, 0.0);
    a.assign(N, 0.0);
    is_valid.assign(N, true);

    v[0] = std::clamp(v0, v_min, v_max);

    for (int i = 0; i < N - 1; ++i) {
        const double v0i = v[i];
        const double k0  = kappa[i];

        const double a_av_max = longAvail(v0i, k0, a_acc_max, a_lat_max);      // >=0
        const double a_av_min = -longAvail(v0i, k0, a_dec_max, a_lat_max);     // <=0

        const double v_sat_next = vsat[i+1];
        const double a_to_sat = (v_sat_next*v_sat_next - v0i*v0i) / (2.0*ds);

        if (a_to_sat >= a_av_max) {
            a[i] = a_av_max;
            
         //   if(i == 0) std::cout << "Forward pass: case 1 triggered at i=0" << std::endl;
         //   if(i == 0) std::cout << "  a_av_max: " << a_av_max << ", a_to_sat: " << a_to_sat << std::endl;
            v[i+1] = safeSqrt(v0i*v0i + 2.0*a[i]*ds);
            v[i+1] = std::min(v[i+1], v_sat_next);
            is_valid[i] = true;
        }
        else if (a_to_sat >= a_av_min && a_to_sat < a_av_max) {
            a[i] = a_to_sat;
            //if(i == 0 ) std::cout << "Forward pass: case 2 triggered at i=0" << std::endl;
            v[i+1] = v_sat_next;
            is_valid[i] = true;
        }
        else {
            a[i] = 0.0;
            //if(i == 0 ) std::cout << "Forward pass: case 3 triggered at i=0" << std::endl;
            v[i+1] = v_sat_next;
            is_valid[i] = false;
        }

        v[i+1] = std::clamp(v[i+1], v_min, v_max);
        v[i+1] = std::min(v[i+1], vsat[i+1]);
    }

    a[N-1] = a[N-2];
}

// backward: your idea (keep a_avg if feasible else max brake)
static inline void fullBackwardPass_fix(
    const std::vector<double>& kappa,
    const std::vector<double>& vsat,
    double ds,
    double a_acc_max,
    double a_dec_max,
    double a_lat_max,
    std::vector<double>& v,
    std::vector<double>& a,
    std::vector<bool>& is_valid)
{
    const int N = (int)kappa.size();
    if ((int)v.size() != N || (int)a.size() != N) return;

    for (int i = N - 1; i >= 1; --i) {
        const double k0 = kappa[i-1];
        const double k1 = kappa[i];

        const double v1 = v[i];
        const double v0_old = v[i-1];

        const double a_avg = (v1*v1 - v0_old*v0_old) / (2.0*ds);
        const bool a_matches = std::abs(a[i-1] - a_avg) < 1e-6;

        if (is_valid[i-1] && a_matches) continue;

        double a1_min, a1_max;
        accelBoundsAt(v1, k1, a_acc_max, a_dec_max, a_lat_max, a1_min, a1_max);

        const double v0_reach_max = safeSqrt(v1*v1 - 2.0*a1_min*ds);                 // a1_min<=0 -> plus
        const double v0_reach_min = safeSqrt(std::max(0.0, v1*v1 - 2.0*a1_max*ds));   // a1_max>=0 -> minus

        double a0_min, a0_max;
        accelBoundsAt(v0_old, k0, a_acc_max, a_dec_max, a_lat_max, a0_min, a0_max);

        const bool valid_v = (v0_old >= v0_reach_min - 1e-9) && (v0_old <= v0_reach_max + 1e-9);
        const bool valid_a_end   = (a_avg >= a1_min - 1e-9) && (a_avg <= a1_max + 1e-9);
        const bool valid_a_start = (a_avg >= a0_min - 1e-9) && (a_avg <= a0_max + 1e-9);

        if (valid_v && valid_a_end && valid_a_start) {
            a[i-1] = a_avg;
            is_valid[i-1] = true;
            continue;
        }

        const double a_brake = -longAvail(v1, k1, a_dec_max, a_lat_max); // <=0

        double v0_new = safeSqrt(v1*v1 - 2.0*a_brake*ds);

        v[i-1] = v0_new;
        a[i-1] = (v1*v1 - v0_new*v0_new) / (2.0*ds);
        is_valid[i-1] = true;
    }

    a[N-1] = a[N-2];
}

// jerk clamp: dt = 2*ds/(v0+v1)
static inline void jerkForwardClamp(
    const std::vector<double>& kappa,
    const std::vector<double>& vsat,
    double ds,
    double a_acc_max,
    double a_dec_max,
    double a_lat_max,
    double jerk_up,
    double jerk_down,
    double a0_along,
    std::vector<double>& v,
    std::vector<double>& a,
    std::vector<bool>& is_valid)
{
    const int N = (int)kappa.size();
    if ((int)v.size() != N || (int)a.size() != N) return;

    const double j_up   = std::max(0.0, jerk_up);
    const double j_down = std::max(0.0, jerk_down);

    double a_prev = std::clamp(a0_along, -a_dec_max, a_acc_max);

    for (int i = 0; i < N - 1; ++i) {
        const double v0 = v[i];
        const double k0 = kappa[i];

        double a_min, a_max;
        accelBoundsAt(v0, k0, a_acc_max, a_dec_max, a_lat_max, a_min, a_max);

        double v1_guess = v[i+1];
        if (!std::isfinite(v1_guess)) v1_guess = v0;

        const double v_sum = std::max(1e-3, v0 + v1_guess);
        const double dt = (2.0*ds) / v_sum;

        const double a_jmin = a_prev - j_down * dt;
        const double a_jmax = a_prev + j_up   * dt;

        double ai = a[i];
        if (!std::isfinite(ai)) ai = 0.0;

        ai = std::clamp(ai, a_jmin, a_jmax);
        ai = std::clamp(ai, a_min, a_max);

        double v1 = safeSqrt(v0*v0 + 2.0*ai*ds);

        if (v1 > vsat[i+1]) {
            is_valid[i] = false;
            v1 = vsat[i+1];
            ai = (v1*v1 - v0*v0) / (2.0*ds);
        }

        a[i] = ai;
        v[i+1] = v1;
        a_prev = ai;
    }

    a[N-1] = a[N-2];
}

static inline void final_jerkForwardClamp(
    const std::vector<double>& kappa,
    const std::vector<double>& vsat,
    double ds,
    double a_acc_max,
    double a_dec_max,
    double a_lat_max,
    double jerk_up,
    double jerk_down,
    double a0_along,
    std::vector<double>& v,
    std::vector<double>& a,
    std::vector<bool>& is_valid){

        const int N = (int)kappa.size();
    if ((int)v.size() != N || (int)a.size() != N) return;

    const double j_up   = std::max(0.0, jerk_up);
    const double j_down = std::max(0.0, jerk_down);

    double a_prev = std::clamp(a0_along, -a_dec_max, a_acc_max);

    for (int i = 0; i < N - 1; ++i) {
        const double v0 = v[i];
        const double k0 = kappa[i];

        double a_min, a_max;
        accelBoundsAt(v0, k0, a_acc_max, a_dec_max, a_lat_max, a_min, a_max);

        double v1_guess = v[i+1];
        if (!std::isfinite(v1_guess)) v1_guess = v0;

        const double v_sum = std::max(1e-3, v0 + v1_guess);
        const double dt = (2.0*ds) / v_sum;

        const double a_jmin = a_prev - j_down * dt;
        const double a_jmax = a_prev + j_up   * dt;

        double ai = a[i];
        if (!std::isfinite(ai)) ai = 0.0;

        ai = std::clamp(ai, a_jmin, a_jmax);
        ai = std::clamp(ai, a_min, a_max);

        double v1 = safeSqrt(v0*v0 + 2.0*ai*ds);

      

        a[i] = ai;
        v[i+1] = v1;
        a_prev = ai;
    }

    a[N-1] = a[N-2];


        
    }

// main planner: forward/backward + (jerk clamp -> FULL backward) merges
static inline SpeedProfileGeom forward_backward_pass_with_jerk_full_backward(
    const TrackSpline2D& sp_closed,
    double s0,
    double ds_geom,
    double S_plan,
    double v0_along,
    double v_min,
    double v_max,
    double a_acc_max,
    double a_dec_max,  // positive magnitude
    double a_lat_max,
    double jerk_up,
    double jerk_down,
    double a0_along,
    int merge_iter)
{

    //std::cout << "along spline distance profile:" <<s0 << std::endl;
    SpeedProfileGeom prof;
    prof.s0 = s0;
    prof.ds = ds_geom;
    prof.S_plan = S_plan;

    const double L = sp_closed.totalLength();
    if (!sp_closed.valid() || !sp_closed.isClosed() || L <= 1e-6) return prof;
    if (ds_geom <= 1e-6 || S_plan <= ds_geom) return prof;

    const int N = (int)std::floor(S_plan / ds_geom) + 1;

    prof.v.assign(N, 0.0);
    prof.kappa.assign(N, 0.0);
    prof.a.assign(N, 0.0);

    for (int i = 0; i < N; ++i) {
        const double s = wrapS(s0 + (double)i * ds_geom, L);
        prof.kappa[i] = sp_closed.getCurvature(s);
    }

    const std::vector<double> vsat = buildVSat(prof.kappa, v_max, a_lat_max);

    std::vector<double> v, a;
    std::vector<bool> is_valid;

    forwardPass_threeCases(
        prof.kappa, vsat,
        ds_geom,
        v_min, v_max,
        a_acc_max, a_dec_max, a_lat_max,
        v0_along,
        v, a, is_valid);

    fullBackwardPass_fix(
        prof.kappa, vsat,
        ds_geom,
        a_acc_max, a_dec_max, a_lat_max,
        v, a, is_valid);

    for (int it = 0; it < std::max(0, merge_iter); ++it) {
        jerkForwardClamp(
            prof.kappa, vsat,
            ds_geom,
            a_acc_max, a_dec_max, a_lat_max,
            jerk_up, jerk_down,
            a0_along,
            v, a,is_valid);


        fullBackwardPass_fix(
            prof.kappa, vsat,
            ds_geom,
            a_acc_max, a_dec_max, a_lat_max,
            v, a, is_valid);
    }
    final_jerkForwardClamp(
        prof.kappa, vsat,
        ds_geom,
        a_acc_max, a_dec_max, a_lat_max,
        jerk_up*5.0, jerk_down*5.0,
        a0_along,
        v, a,is_valid);

    // final clamps
    for (int i = 0; i < N; ++i) {
        double vi = v[i];
        if (!std::isfinite(vi)) vi = 0.0;
        vi = std::clamp(vi, v_min, v_max);
        vi = std::min(vi, vsat[i]);
        v[i] = vi;
    }
    for (int i = 0; i < N; ++i) {
        if (!std::isfinite(a[i])) a[i] = 0.0;
    }

    prof.v = std::move(v);
    prof.a = std::move(a);

    return prof;
}

struct Profile_at_single_point
{
    double s0   = 0.0;
    double a_ref = 0.0;
    double v_ref = 0.0;
    double k_ref = 0.0;
    double R_ref = 0.0;
};

static inline Profile_at_single_point profile_atDistance(const SpeedProfileGeom& prof, double dist)
{
    Profile_at_single_point res;
    double distance_temp = wrapS(dist, prof.S_plan);
    res.s0 = distance_temp;
    const int N = (int)prof.v.size();
    if (N < 2) return res;
    if ((int)prof.a.size() != N) return res;
    if ((int)prof.kappa.size() != N) return res;

    double dist_from_profile_start = dist - prof.s0;
    dist_from_profile_start = wrapS(dist_from_profile_start, prof.S_plan);
    
    const double u = dist_from_profile_start / prof.ds;
    int i = (int)std::floor(u);
    i = std::clamp(i, 0, N - 2);
    

    const double alpha = u - (double)i;
    //std::cout<<"alpha: "<<alpha<<std::endl;

    auto lerp = [&](double x0, double x1) -> double {
        return (1.0 - alpha)*x0 + alpha*x1;
    };

    res.v_ref = lerp(prof.v[i],     prof.v[i+1]);
    res.a_ref = lerp(prof.a[i],     prof.a[i+1]);
    // if(i == 0 ){
    //     std::cout << "a at 0:" << prof.a[i] << " a at 1 :"<< prof.a[i+1] <<std::endl;
    //     std::cout << "a ref: " << res.a_ref << std::endl;
    // }
    res.k_ref = lerp(prof.kappa[i], prof.kappa[i+1]);

    const double kk = std::abs(res.k_ref);
    res.R_ref = (kk > 1e-9) ? (1.0 / kk) : std::numeric_limits<double>::infinity();

    return res;
}

static inline void logFrictionUseEllipse_Planned(
    int k,
    double v_plan,
    double kappa_plan,
    double a_lat_plan,
    double a_long_plan,
    double a_acc_max,
    double a_dec_max,
    double a_lat_max
)
{
    const double lat_den  = std::max(1e-9, a_lat_max);
    const double long_den = std::max(1e-9, (a_long_plan >= 0.0 ? a_acc_max : a_dec_max));

    const double u_lat  = a_lat_plan  / lat_den;
    const double u_long = a_long_plan / long_den;

    const double usage = u_lat*u_lat + u_long*u_long;

    (void)k;
    (void)v_plan;
    (void)kappa_plan;
    (void)usage;
    (void)u_lat;
    (void)u_long;
}

// =====================================================
// FINAL: all_path + velocity planner (closed loop)
// =====================================================
inline PathProcessResult all_path_and_velocity_planner_process_for_control(
    const ParamBank& P,
    const TrackSpline2D& spline_closed,
    const State& bolide_state)
{
    PathProcessResult out;

    if (!spline_closed.valid() || !spline_closed.isClosed()) {
        ROS_WARN_STREAM("[VelPlanner] spline invalid or not closed");
        return out;
    }

    const double L = spline_closed.totalLength();
    const int    K = (int)P.get("mpc_N");

    if (K < 2 || L <= 1e-6) {
        ROS_WARN_STREAM("[VelPlanner] bad K or L");
        return out;
    }

    const double ds_geom = P.get("vel_planner_spatial_step");
    const double ds_mpc  = P.get("mpc_spatial_step");

    const double a_acc_max_raw = P.get("vel_planner_max_accel");                 // +
    const double a_dec_max_raw = P.get("vel_planner_max_decel");                 // + (magnitude)
    const double v_min_raw     = P.get("vel_planner_v_min");
    const double v_max_raw     = P.get("vel_planner_v_max");
    const double a_lat_max_raw = P.get("vel_planner_max_corrnering_accel");
    const double safety        = P.get("vel_planner_saftey_factor");

    const double j_max_raw = P.get("vel_planner_max_jerk");
    const int merge_iter = (int)P.get("vel_planner_number_of_jerk_merging_iterations");

    const double a_acc_max = std::max(0.0, safety * a_acc_max_raw);
    const double a_dec_max = std::max(0.0, safety * std::abs(a_dec_max_raw));
    const double a_lat_max = std::max(1e-6, safety * a_lat_max_raw);

    const double v_min = std::max(0.0, v_min_raw);
    const double v_max = std::max(v_min + 1e-3,  v_max_raw);

    const double j_max = std::max(0.0,  j_max_raw);

    double mpc_dt = 1.0/P.get("odom_frequency");

    // ---- find s0 (coarse + project) ----
    const Vec2 q((float)bolide_state.X, (float)bolide_state.Y);

    double s_guess = 0.0;
    {
        const int M = std::max(60, (int)std::ceil(L / 0.5));
        double best_d2 = std::numeric_limits<double>::infinity();
        for (int i = 0; i < M; ++i) {
            const double s = (double)i * (L / (double)M);
            const Vec2 p = spline_closed.eval(s);
            const double dx = (double)p.x - (double)q.x;
            const double dy = (double)p.y - (double)q.y;
            const double d2 = dx*dx + dy*dy;
            if (d2 < best_d2) { best_d2 = d2; s_guess = s; }
        }
    }
    const double s0 = spline_closed.projectToS(q, s_guess, 8.0, 61, 8);

    // ---- yaw0 ----
    const double yaw0 = spline_closed.getYaw(s0);
    out.yaw0 = yaw0;

    // ---- v0 along ----
    //double v0_along = bolide_state.vx * std::cos(bolide_state.yaw - yaw0) - bolide_state.vy * std::sin(bolide_state.yaw - yaw0);
    //if (!std::isfinite(v0_along)) v0_along = 0.0;
    double v0_along = bolide_state.vx;
    v0_along = std::clamp(v0_along, v_min, v_max);

    // ---- a0 along ----
    double a0_along = bolide_state.acc;
    if (!std::isfinite(a0_along)) a0_along = 0.0;
    a0_along = std::clamp(a0_along, -a_dec_max, a_acc_max);

    // ---- build full-loop speed profile ----
    const double S_plan = L;
    SpeedProfileGeom prof = forward_backward_pass_with_jerk_full_backward(
        spline_closed,
        s0,
        ds_geom,
        S_plan,
        v0_along,
        v_min,
        v_max,
        a_acc_max,
        a_dec_max,
        a_lat_max,
        j_max,
        j_max,
        a0_along,
        merge_iter
    );

    if (prof.v.size() < 2) {
        ROS_WARN_STREAM("[VelPlanner] failed to build speed profile");
        return out;
    }

    

    // ---- output refs for MPC horizon ----
    out.N = K;
    out.X_ref.resize(K);
    out.Y_ref.resize(K);
    out.curvature.resize(K);
    out.velocity_ref.resize(K);
    out.acceleration_ref.resize(K);

    out.valid = true;
    out.geo_eligible = true;
    out.curv_eligible = true;
    out.mpc_eligible = true;
    out.all_path_eligible = true;

    double s_along = s0;
    // std::cout << "Initial along spline distance for MPC: " << s_along << std::endl;
    // std::cout << "Spline total length: " << L << std::endl;
    for (int i = 0; i < K; ++i) {

        s_along = wrapS(s_along, L);
        // if(i == 0) std::cout << "along spline distance  sampling:" <<s_along << std::endl;

        const auto pr = profile_atDistance(prof, s_along);        
        out.X_ref(i) = spline_closed.getX(s_along);
        out.Y_ref(i) = spline_closed.getY(s_along);
        out.curvature(i) = spline_closed.getCurvature(s_along);
        out.velocity_ref(i) = pr.v_ref;
        out.acceleration_ref(i) = pr.a_ref;
        if(i == 0) {
           // std::cout<< "a ref at first MPC point: " << pr.a_ref << std::endl;
        }
        s_along += pr.v_ref*mpc_dt + 0.5*pr.a_ref*mpc_dt*mpc_dt;
    }

    return out;
}

// =====================================================
// Główna funkcja (NIE RUSZONA LOGIKA PATH/MPC) + hook all_path
// =====================================================
inline PathProcessResult path_process_for_control(
        const ParamBank& P,
        const Eigen::VectorXd& X_path,
        const Eigen::VectorXd& Y_path,
        const State& bolide_state,
        bool all_path_eligible_flag = false)
{
    if (X_path.size() != Y_path.size())
    {
        ROS_WARN_STREAM("[PathProcessing] INVALID: X_path.size() != Y_path.size()");
        return PathProcessResult{};
    }

    const int Nraw = static_cast<int>(X_path.size());

    if (Nraw < 2)
        return makeGeoInvalidResult(X_path, Y_path);

    if (Nraw < 3)
    {
        float ds = static_cast<float>(P.get("distance_between_interpoleted_points"));
        const auto p0_x = X_path(0);
        const auto p0_y = Y_path(0);
        const auto p1_x = X_path(1);
        const auto p1_y = Y_path(1);

        const double dx = p1_x - p0_x;
        const double dy = p1_y - p0_y;
        double L = std::hypot(dx, dy);
        if (L < 1e-9) {
            Eigen::VectorXd X_tmp(2), Y_tmp(2);
            X_tmp << p0_x, p0_x;
            Y_tmp << p0_y, p0_y;
            return makeCurvInvalidResult(X_tmp, Y_tmp);
        }

        double bx = bolide_state.X;
        double by = bolide_state.Y;

        double wx = bx - p0_x;
        double wy = by - p0_y;

        double dot_vv = dx*dx + dy*dy;
        double t = (wx*dx + wy*dy) / dot_vv;

        double dist_to_p1 = -(t - 1.0) * L;

        double L_new = L;
        if(dist_to_p1 < P.get("min_path_length_for_geo") && t <= 1 && t > 0)
            L_new += P.get("min_path_length_for_geo") - dist_to_p1;
        if(t > 1)
            L_new += P.get("min_path_length_for_geo") - dist_to_p1;

        size_t pts_number = static_cast<size_t>(std::ceil(L / ds)) + 1;

        Eigen::VectorXd X_tmp(pts_number);
        Eigen::VectorXd Y_tmp(pts_number);

        double scale = L_new / L;
        for (size_t i = 0; i < pts_number; ++i) {
            double tp = static_cast<double>(i) / static_cast<double>(pts_number - 1);
            X_tmp(i) = p0_x + scale*tp * dx;
            Y_tmp(i) = p0_y + scale*tp * dy;
        }
        return makeCurvInvalidResult(X_tmp, Y_tmp);
    }

    if (all_path_eligible_flag)
    {
        std::vector<Vec2> path_base = buildPathBase(X_path, Y_path);

        TrackSpline2D spline_closed;
        spline_closed.build(path_base, true);

        if (!spline_closed.valid()) {
            ROS_WARN_STREAM("[PathProcessing] Closed spline build failed.");
            return PathProcessResult{};
        }

        return all_path_and_velocity_planner_process_for_control(P, spline_closed, bolide_state);
    }

    PathProcessResult result;

    std::vector<Vec2> path_base = buildPathBase(X_path, Y_path);

    TrackSpline2D spline_open;
    spline_open.build(path_base, false);

    if (!spline_open.valid()) {
        ROS_WARN_STREAM("[PathProcessing] Open spline build failed. Returning non-mpc eligible polyline.");
        const int K = static_cast<int>(path_base.size());
        result.N = K;
        result.X_ref.resize(K);
        result.Y_ref.resize(K);
        result.velocity_ref = Eigen::VectorXd::Constant(K, P.get("v_target"));
        result.acceleration_ref = Eigen::VectorXd::Zero(K);
        result.valid = true;
        result.geo_eligible = true;
        result.curv_eligible = true;
        result.mpc_eligible = false;
        result.all_path_eligible = false;

        for (int i = 0; i < K; ++i) {
            result.X_ref(i) = path_base[i].x;
            result.Y_ref(i) = path_base[i].y;
        }
        result.curvature = computeCurvature(path_base);

        if (K >= 2) result.yaw0 = yawFrom2Pts(result.X_ref(0), result.Y_ref(0), result.X_ref(1), result.Y_ref(1));
        else        result.yaw0 = 0.0;

        return result;
    }

    const double ds = P.get("distance_between_interpoleted_points");
    const size_t max_pts = static_cast<size_t>(P.get("interpoleted_num_max_points"));
    std::vector<Vec2> path_dense = sampleSplineByS(spline_open, ds, max_pts);

    if (path_dense.size() < 2) {
        ROS_WARN_STREAM("[PathProcessing] Dense path sampling failed. Returning non-mpc eligible.");
        const int K = static_cast<int>(path_base.size());
        result.N = K;
        result.X_ref.resize(K);
        result.Y_ref.resize(K);
        result.velocity_ref = Eigen::VectorXd::Constant(K, P.get("v_target"));
        result.acceleration_ref = Eigen::VectorXd::Zero(K);
        result.valid = true;
        result.geo_eligible = true;
        result.curv_eligible = true;
        result.mpc_eligible = false;
        result.all_path_eligible = false;

        for (int i = 0; i < K; ++i) {
            result.X_ref(i) = path_base[i].x;
            result.Y_ref(i) = path_base[i].y;
        }
        result.curvature = computeCurvature(path_base);

        if (K >= 2) result.yaw0 = yawFrom2Pts(result.X_ref(0), result.Y_ref(0), result.X_ref(1), result.Y_ref(1));
        else        result.yaw0 = 0.0;

        return result;
    }

    Vec2 Pbol(bolide_state.X, bolide_state.Y);
    ProjectionInfo proj = findProjectionOnPath(path_dense, Pbol, 30);

    double remaining_len = computeRemainingPathLength(path_dense, proj.index, proj.point);
    double min_horizon = P.get("mpc_min_horizon_meters_length");

    if (remaining_len >= min_horizon)
    {
        if (proj.index == 0) {
            return processMpcEligiblePath_bolide_behind_path(spline_open, proj, P);
        } else {
            return processMpcEligiblePath_bolide_on_path(spline_open, path_dense, proj, P);
        }
    }
    else
    {
        const int K = static_cast<int>(path_dense.size());
        result.N = K;
        result.X_ref.resize(K);
        result.Y_ref.resize(K);
        result.velocity_ref = Eigen::VectorXd::Constant(K, P.get("v_target"));
        result.acceleration_ref = Eigen::VectorXd::Zero(K);
        result.valid = true;
        result.geo_eligible = true;
        result.curv_eligible = true;
        result.mpc_eligible = false;
        result.all_path_eligible = false;

        for (int i = 0; i < K; ++i) {
            result.X_ref(i) = path_dense[i].x;
            result.Y_ref(i) = path_dense[i].y;
        }

        result.curvature = computeCurvature(path_dense);

        if (K >= 2) result.yaw0 = yawFrom2Pts(result.X_ref(0), result.Y_ref(0), result.X_ref(1), result.Y_ref(1));
        else        result.yaw0 = 0.0;

        return result;
    }
}

} // namespace v2_control
