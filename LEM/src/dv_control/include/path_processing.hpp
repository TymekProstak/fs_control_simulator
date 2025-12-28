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

    // yaw ścieżki na początku horyzontu (w MAP)
    double yaw0 = 0.0;
};

// =====================================================
// Helper: bezpieczny yaw z dwóch punktów
// =====================================================
static inline double yawFrom2Pts(double x0, double y0, double x1, double y1)
{
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double n2 = dx*dx + dy*dy;
    if (n2 < 1e-12) return 0.0;
    return std::atan2(dy, dx);
}

// =====================================================
// Helper: rzut punktu na segment
// =====================================================
inline Vec2 projectPointOnSegment(const Vec2& A, const Vec2& B, const Vec2& P)
{
    Vec2 AB = B - A;
    double len2 = AB.x * AB.x + AB.y * AB.y;

    if (len2 < 1e-12)
        return A;

    double t = ((P.x - A.x) * AB.x + (P.y - A.y) * AB.y) / len2;
    t = std::clamp(t, 0.0, 1.0);

    return Vec2(A.x + t * AB.x, A.y + t * AB.y);
}

// =====================================================
// Wyniki "za krótka ścieżka"
// =====================================================
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

// =====================================================
// Buduję Vec2 z X/Y
// =====================================================
inline std::vector<Vec2> buildPathBase(const Eigen::VectorXd& X,
                                      const Eigen::VectorXd& Y)
{
    const int N = static_cast<int>(X.size());
    std::vector<Vec2> path;
    path.reserve(N);
    for (int i = 0; i < N; ++i) {
        path.emplace_back(X(i), Y(i));
    }
    return path;
}

// =====================================================
// Krzywizna z 3 punktów (fallback / debug)
// =====================================================
inline Eigen::VectorXd computeCurvature(const std::vector<Vec2>& path)
{
    const int K = static_cast<int>(path.size());
    Eigen::VectorXd kappa = Eigen::VectorXd::Zero(K);

    if (K < 3)
        return kappa;

    for (int i = 1; i < K - 1; ++i) {
        double x0 = path[i - 1].x;
        double y0 = path[i - 1].y;
        double x1 = path[i].x;
        double y1 = path[i].y;
        double x2 = path[i + 1].x;
        double y2 = path[i + 1].y;

        double vx1 = x1 - x0;
        double vy1 = y1 - y0;
        double vx2 = x2 - x1;
        double vy2 = y2 - y1;

        double a = std::hypot(vx1, vy1);
        double b = std::hypot(vx2, vy2);
        double c = std::hypot(x2 - x0, y2 - y0);

        double cross = vx1 * vy2 - vy1 * vx2;
        double denom = a * b * c;

        kappa(i) = (denom > 1e-9) ? 2.0 * cross / denom : 0.0;
    }

    kappa(0)     = kappa(1);
    kappa(K - 1) = kappa(K - 2);

    return kappa;
}

// =====================================================
// gęste punkty po SPLAJNIE: próbkowanie eval(s) co ds
// =====================================================
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
        double s = std::min(L, (double)i * ds);
        out.push_back(sp.eval(s));
    }

    if (!sp.isClosed()) {
        if (out.empty() ||
            std::hypot(out.back().x - sp.eval(L).x, out.back().y - sp.eval(L).y) > 1e-6)
        {
            if (max_points == 0 || out.size() < max_points)
                out.push_back(sp.eval(L));
        }
    }
    return out;
}

// =====================================================
// Projekcja bolidu na polyline (segmenty)
// =====================================================
struct ProjectionInfo {
    int index = 0; // i = punkt "B" segmentu [i-1, i]
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

    Vec2 A0 = path[0];
    Vec2 B0 = path[1];
    Vec2 AB0 = B0 - A0;
    double len2_0 = AB0.x * AB0.x + AB0.y * AB0.y;

    if (len2_0 >= 1e-6) {
        double t0 = ((Pbol.x - A0.x) * AB0.x + (Pbol.y - A0.y) * AB0.y) / len2_0;
        if (t0 < 0.0) {
            info.point = Vec2(A0.x + t0 * AB0.x, A0.y + t0 * AB0.y);
            info.index = 0;
            return info;
        }
    }

    const int N = static_cast<int>(path.size());
    int search_limit = std::min(N - 1, maxSegmentsToSearch);

    double best_d2 = 1e30;
    for (int i = 1; i <= search_limit; ++i) {
        Vec2 A = path[i - 1];
        Vec2 B = path[i];
        Vec2 Pp = projectPointOnSegment(A, B, Pbol);

        double dx = Pp.x - Pbol.x;
        double dy = Pp.y - Pbol.y;
        double d2 = dx * dx + dy * dy;

        if (d2 < best_d2) {
            best_d2 = d2;
            info.point = Pp;
            info.index = i;
        }
    }

    return info;
}

// =====================================================
// Remaining length (open polyline)
// =====================================================
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
        double dx = path[i + 1].x - path[i].x;
        double dy = path[i + 1].y - path[i].y;
        dist += std::hypot(dx, dy);
    }

    return dist;
}

// =====================================================
// s_guess z geometrii polyline (dla projectToS)
// =====================================================
inline double guessSFromProjection(const std::vector<Vec2>& path,
                                  const ProjectionInfo& proj)
{
    if (path.size() < 2) return 0.0;

    int i = std::clamp(proj.index, 1, (int)path.size() - 1);

    double s = 0.0;
    for (int k = 0; k < i - 1; ++k) {
        double dx = path[k + 1].x - path[k].x;
        double dy = path[k + 1].y - path[k].y;
        s += std::hypot(dx, dy);
    }

    double dxp = proj.point.x - path[i - 1].x;
    double dyp = proj.point.y - path[i - 1].y;
    s += std::hypot(dxp, dyp);

    return s;
}

// =====================================================
// MPC-eligible: bolid behind path (open)
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

    double dx0 = spline.getX(0.0) - projection.point.x;
    double dy0 = spline.getY(0.0) - projection.point.y;

    double dist_to_first_path_point = std::hypot(dx0, dy0);
    dist_to_first_path_point = std::max(dist_to_first_path_point, 1e-9);

    int points_to_add = static_cast<int>(std::ceil(dist_to_first_path_point / P.get("mpc_spatial_step")));
    points_to_add = std::max(1, points_to_add - 1);

    double eps = 1e-6;
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

    double dist_from_last_added_point_to_spline_start =
        -dist_to_first_path_point + (points_to_add - 1) * P.get("mpc_spatial_step");

    double s = std::max(dist_from_last_added_point_to_spline_start + P.get("mpc_spatial_step"), 0.0);

    for (int i = points_to_add; i < K; i++) {
        X_temp(i) = spline.getX(s);
        Y_temp(i) = spline.getY(s);
        curv_temp_ref(i) = spline.getCurvature(s);

        s += P.get("mpc_spatial_step");
        if (!spline.isClosed() && s > spline.totalLength())
            s = spline.totalLength();
    }

    result.X_ref = X_temp;
    result.Y_ref = Y_temp;
    result.curvature = curv_temp_ref;

    if (K >= 2) result.yaw0 = yawFrom2Pts(result.X_ref(0), result.Y_ref(0), result.X_ref(1), result.Y_ref(1));
    else        result.yaw0 = 0.0;

    return result;
}

// =====================================================
// MPC-eligible: bolid on path (open)
// =====================================================
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

    double s_guess = guessSFromProjection(path_dense_for_guess, projection);
    double s = spline.projectToS(projection.point, s_guess, 2.0, 41, 6);

    result.yaw0 = spline.getYaw(s);
    result.curvature(0) = spline.getCurvature(s);

    for (int i = 1; i < K; ++i) {
        s += P.get("mpc_spatial_step");
        if (!spline.isClosed() && s > spline.totalLength())
            s = spline.totalLength();

        result.X_ref(i) = spline.getX(s);
        result.Y_ref(i) = spline.getY(s);
        result.curvature(i) = spline.getCurvature(s);
    }

    return result;
}

/// ====================================================
/// ALL_PATH_ELIGIBLE: planner prędkości + closed path
/// ====================================================

struct SpeedProfileGeom
{
    double s0 = 0.0;
    double ds = 0.5;
    double S_plan = 0.0;
    std::vector<double> v;
    std::vector<double> kappa;
};

static inline double wrapS(double s, double L)
{
    if (L <= 1e-12) return 0.0;
    s = std::fmod(s, L);
    if (s < 0.0) s += L;
    return s;
}

static inline double wrapDist(double d, double L)
{
    if (L <= 1e-12) return 0.0;
    d = std::fmod(d, L);
    if (d < 0.0) d += L;
    return d;
}

static inline double longAvail(double v, double kappa, double a_long_max, double a_lat_max)
{
    const double ay = v*v * std::abs(kappa);
    const double r  = std::min(1.0, ay / std::max(1e-9, a_lat_max));
    const double term = std::max(0.0, 1.0 - r*r);
    return a_long_max * std::sqrt(term);
}

static inline SpeedProfileGeom buildSpeedProfileGeom(
    const TrackSpline2D& sp_closed,
    double s0,
    double ds_geom,
    double S_plan,
    double v0_along,
    double v_min,
    double v_max,
    double a_acc_max,
    double a_dec_max,
    double a_lat_max
)
{
    (void)v_min;

    SpeedProfileGeom prof;
    prof.s0 = s0;
    prof.ds = ds_geom;
    prof.S_plan = S_plan;

    const double L = sp_closed.totalLength();
    if (!sp_closed.valid() || !sp_closed.isClosed() || L <= 1e-6) return prof;
    if (ds_geom <= 1e-6 || S_plan <= ds_geom) return prof;

    const int N = (int)std::floor(S_plan / ds_geom) + 1;
    prof.v.resize(N);
    prof.kappa.resize(N);

    std::vector<double> v_lim(N);

    for (int i = 0; i < N; ++i) {
        double s = wrapS(s0 + (double)i * ds_geom, L);
        double kap = sp_closed.getCurvature(s);
        prof.kappa[i] = kap;

        const double kk = std::abs(kap);
        double v_kappa = v_max;
        if (kk > 0.001) v_kappa = std::sqrt(a_lat_max / kk);

        v_lim[i] = std::clamp(v_kappa, 0.0, v_max);
    }

    const double v0 = std::clamp(v0_along, 0.0, v_max);

    std::vector<double> v_fwd(N);
    v_fwd[0] = std::min(v_lim[0], v0);

    for (int i = 0; i < N - 1; ++i) {
        const double a_av = longAvail(v_fwd[i], prof.kappa[i], a_acc_max, a_lat_max);
        const double v_next_max = std::sqrt(std::max(0.0, v_fwd[i]*v_fwd[i] + 2.0*a_av*ds_geom));
        v_fwd[i+1] = std::min(v_lim[i+1], v_next_max);
    }

    std::vector<double> v_bwd(N);
    v_bwd[N-1] = v_fwd[N-1];

    for (int i = N - 2; i >= 0; --i) {
        const double a_av = longAvail(v_bwd[i+1], prof.kappa[i+1], a_dec_max, a_lat_max);
        const double v_prev_max = std::sqrt(std::max(0.0, v_bwd[i+1]*v_bwd[i+1] + 2.0*a_av*ds_geom));
        v_bwd[i] = std::min(v_lim[i], v_prev_max);
    }

    for (int i = 0; i < N; ++i) {
        prof.v[i] = std::min(v_fwd[i], v_bwd[i]);
        prof.v[i] = std::clamp(prof.v[i], 0.0, v_lim[i]);
    }

    prof.v[0] = std::min(prof.v[0], v0);
    return prof;
}

static inline double profileV_atDistance(const SpeedProfileGeom& prof, double dist)
{
    if (prof.v.size() < 2) return 0.0;

    dist = std::clamp(dist, 0.0, prof.S_plan);
    const double u = dist / prof.ds;
    int i = (int)std::floor(u);
    i = std::clamp(i, 0, (int)prof.v.size() - 2);

    const double a = u - (double)i;
    return (1.0 - a) * prof.v[i] + a * prof.v[i+1];
}

// =====================================================
// friction usage (elipsa)
// =====================================================
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
// - START PROFILU z v_along z odometrii (bez wymuszenia v_min)
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

    const double odom_freq = P.get("odom_frequency");
    const double dt = (odom_freq > 1e-6) ? (1.0 / odom_freq) : 0.02;

    double ds_geom = 0.3;

    const double a_acc_max_raw = P.get("vel_planner_max_accel");
    const double a_dec_max_raw = -P.get("vel_planner_max_decel");
    const double v_min         = P.get("vel_planner_v_min");
    const double v_max         = P.get("vel_planner_v_max");
    const double a_lat_max_raw = P.get("vel_planner_max_corrnering_accel");
    const double safety        = P.get("vel_planner_saftey_factor");

    const double a_acc_max = std::max(0.0, safety * a_acc_max_raw);
    const double a_dec_max = std::max(0.0, safety * a_dec_max_raw);
    const double a_lat_max = std::max(1e-6, safety * a_lat_max_raw);

    if (K < 2 || L <= 1e-6) {
        ROS_WARN_STREAM("[VelPlanner] bad K or L");
        return out;
    }

    // -------- find s0 --------
    const Vec2 q((float)bolide_state.X, (float)bolide_state.Y);

    double s_guess = 0.0;
    {
        const int M = std::max(60, (int)std::ceil(L / 0.5));
        double best_d2 = std::numeric_limits<double>::infinity();
        for (int i = 0; i < M; ++i) {
            const double s = (double)i * (L / (double)M);
            Vec2 p = spline_closed.eval(s);
            const double dx = (double)p.x - (double)q.x;
            const double dy = (double)p.y - (double)q.y;
            const double d2 = dx*dx + dy*dy;
            if (d2 < best_d2) { best_d2 = d2; s_guess = s; }
        }
    }

    const double s0 = spline_closed.projectToS(q, s_guess, 8.0, 61, 8);

    // -------- v0 along track --------
    const double yaw0 = spline_closed.getYaw(s0);
    out.yaw0 = yaw0; // <<< KLUCZ: zwracamy yaw0

    const double bolide_yaw = bolide_state.yaw;

    double epsi = bolide_yaw - yaw0;
    unwrap_angle(epsi);

    double v0_along = bolide_state.vx * std::cos(epsi) + bolide_state.vy * std::sin(epsi);
    if (!std::isfinite(v0_along)) v0_along = 0.0;

    v0_along = std::clamp(v0_along, 0.0, v_max);

    // -------- build v(dist) over full loop --------
    const double S_plan = L;
    SpeedProfileGeom prof = buildSpeedProfileGeom(
        spline_closed,
        s0,
        ds_geom,
        S_plan,
        v0_along,
        v_min, v_max,
        a_acc_max, a_dec_max, a_lat_max
    );

    if (prof.v.size() < 2) {
        ROS_WARN_STREAM("[VelPlanner] failed to build speed profile");
        return out;
    }

    // -------- time sampling to MPC horizon --------
    out.N = K;
    out.X_ref.resize(K);
    out.Y_ref.resize(K);
    out.curvature.resize(K);
    out.velocity_ref.resize(K);
    out.acceleration_ref.resize(K);

    double s = s0;
    double dist_ahead = 0.0;

    std::vector<double> v_tmp(K, 0.0);
    std::vector<double> kappa_tmp(K, 0.0);
    std::vector<double> a_lat_tmp(K, 0.0);

    double v_prev = std::clamp(v0_along, 0.0, v_max);

    for (int k = 0; k < K; ++k)
    {
        Vec2 p = spline_closed.eval(s);
        out.X_ref(k) = p.x;
        out.Y_ref(k) = p.y;

        const double kappa = spline_closed.getCurvature(s);
        out.curvature(k) = kappa;
        kappa_tmp[k] = kappa;

        dist_ahead = wrapDist(dist_ahead, S_plan);
        const double v_prof = profileV_atDistance(prof, dist_ahead);

        double v;
        if (k == 0) {
            v = v_prev;
        } else {
            const double v_min_next = std::max(0.0, v_prev - a_dec_max * dt);
            const double v_max_next = v_prev + a_acc_max * dt;
            v = std::clamp(v_prof, v_min_next, v_max_next);
            v = std::clamp(v, v_min, v_max);
        }

        out.velocity_ref(k) = v;
        v_tmp[k] = v;

        const double a_lat_plan = v * v * std::abs(kappa);
        a_lat_tmp[k] = a_lat_plan;

        v_prev = v;

        const double ds_k = std::max(v,0.2) * dt;
        dist_ahead += ds_k;
        s = wrapS(s + ds_k, L);
    }

    for (int k = 0; k < K - 1; ++k) {
        out.acceleration_ref(k) = (v_tmp[k+1] - v_tmp[k]) / std::max(1e-6, dt);
    }
    out.acceleration_ref(K-1) = out.acceleration_ref(K-2);

    {
        const int k = 0;
        logFrictionUseEllipse_Planned(
            k,
            v_tmp[k],
            kappa_tmp[k],
            a_lat_tmp[k],
            out.acceleration_ref(k),
            a_acc_max,
            a_dec_max,
            a_lat_max
        );
    }

    out.valid = true;
    out.geo_eligible = true;
    out.curv_eligible = true;
    out.mpc_eligible = true;
    out.all_path_eligible = true;

    return out;
}

// =====================================================
// Główna funkcja
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

    // ===================================================================
    // TRYB ALL_PATH_ELIGIBLE: closed loop + planner prędkości
    // ===================================================================
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

    // ===================================================================
    // TRYB NORMALNY: open path
    // ===================================================================
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
