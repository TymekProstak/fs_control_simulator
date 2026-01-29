#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

#include <ros/ros.h>

#include "spline.hpp"       // TrackSpline2D
#include "ParamBank.hpp"
#include "Vec2.hpp"
#include "mpc_interface.hpp"

namespace v2_control {

// =====================================================
// Forward declarations
// =====================================================
struct Poly_with_kappa;

// =====================================================
// Basic structs
// =====================================================
struct PathProcessResult
{
    int N = 0;

    Eigen::VectorXd X_ref;
    Eigen::VectorXd Y_ref;
    Eigen::VectorXd curvature;

    Eigen::VectorXd acceleration_ref;
    Eigen::VectorXd velocity_ref;

    bool valid = false;
    bool curv_eligible = false;
    bool geo_eligible = false;
    bool mpc_eligible = false;
    bool all_path_eligible = false;

    double yaw0 = 0.0;
};

struct Poly_with_kappa
{
    std::vector<Vec2>   path;
    std::vector<double> kappa;

    size_t size() const { return path.size(); }
};

// =====================================================
// Geometry helpers (small, clean, reusable)
// =====================================================
static inline double yawFrom2Pts(double x0, double y0, double x1, double y1)
{
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double n2 = dx*dx + dy*dy;
    if (n2 < 1e-12) return 0.0;
    return std::atan2(dy, dx);
}

static inline double yawFrom2Pts(const Vec2& p0, const Vec2& p1)
{
    return yawFrom2Pts((double)p0.x, (double)p0.y, (double)p1.x, (double)p1.y);
}

// =====================================================
// Segment projection utilities
// =====================================================
static inline double find_t_on_segment(const Vec2& A, const Vec2& B, const Vec2& P)
{
    const Vec2 AB = B - A;
    const double len2 = AB.x*AB.x + AB.y*AB.y;
    if (len2 < 1e-12) return 0.0;

    const double t = ((P.x - A.x)*AB.x + (P.y - A.y)*AB.y) / len2;
    return t; // ja celowo nie clampuję: t<0 i t>1 jest mi potrzebne do wykrywania “przed startem / po końcu”
}

static inline Vec2 projectPointOnSegment_clamped01(const Vec2& A, const Vec2& B, const Vec2& P)
{
    const Vec2 AB = B - A;
    const double len2 = AB.x*AB.x + AB.y*AB.y;

    if (len2 < 1e-12) return A;

    double t = ((P.x - A.x)*AB.x + (P.y - A.y)*AB.y) / len2;
    t = std::clamp(t, 0.0, 1.0);

    return Vec2(A.x + t*AB.x, A.y + t*AB.y);
}

static inline int find_closest_segment_index(const std::vector<Vec2>& path, const Vec2& point)
{
    if (path.size() < 2) return -1;

    int closest_index = -1;
    double best_d2 = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Vec2 A = path[i];
        const Vec2 B = path[i + 1];

        const Vec2 proj = projectPointOnSegment_clamped01(A, B, point);
        const double dx = proj.x - point.x;
        const double dy = proj.y - point.y;
        const double d2 = dx*dx + dy*dy;

        if (d2 < best_d2) {
            best_d2 = d2;
            closest_index = (int)i;
        }
    }
    return closest_index;
}

// =====================================================
// Conversions
// =====================================================
static inline std::vector<Vec2> eigenToVec2(const Eigen::VectorXd& X, const Eigen::VectorXd& Y)
{
    const int N = (int)X.size();
    std::vector<Vec2> out;
    out.reserve(N);
    for (int i = 0; i < N; ++i) out.emplace_back((float)X(i), (float)Y(i));
    return out;
}

// =====================================================
// Result constructors
// =====================================================
static inline PathProcessResult makeGeoInvalidResult()
{
    PathProcessResult r;
    r.N = 0;
    r.valid = false;
    r.geo_eligible = false;
    r.curv_eligible = false;
    r.mpc_eligible = false;
    r.all_path_eligible = false;
    return r;
}

static inline PathProcessResult makeCurvInvalidResult(
    const Eigen::VectorXd& X,
    const Eigen::VectorXd& Y,
    double yaw0,
    const ParamBank& P)
{
    PathProcessResult r;
    const int N = (int)X.size();

    r.N = N;
    r.X_ref = X;
    r.Y_ref = Y;

    r.curvature = Eigen::VectorXd::Zero(N);
    r.velocity_ref = Eigen::VectorXd::Constant(N, P.get("v_target"));
    r.acceleration_ref = Eigen::VectorXd::Zero(N);

    r.valid = true;
    r.geo_eligible = true;
    r.curv_eligible = false;
    r.mpc_eligible = false;
    r.all_path_eligible = false;

    r.yaw0 = yaw0;
    return r;
}

static inline PathProcessResult makeCurvInvalidResult(
    const std::vector<Vec2>& path,
    double yaw0,
    const ParamBank& P)
{
    const int N = (int)path.size();
    if (N < 2) return makeGeoInvalidResult();

    Eigen::VectorXd X(N), Y(N);
    for (int i = 0; i < N; ++i) {
        X(i) = (double)path[i].x;
        Y(i) = (double)path[i].y;
    }
    return makeCurvInvalidResult(X, Y, yaw0, P);
}

static inline PathProcessResult makeResultFromPolyWithKappa(
    const Poly_with_kappa& poly,
    double yaw0,
    const ParamBank& P)
{
    PathProcessResult r;
    const int N = (int)poly.path.size();
    if (N < 2) return makeGeoInvalidResult();

    r.N = N;
    r.X_ref.resize(N);
    r.Y_ref.resize(N);
    r.curvature.resize(N);

    r.velocity_ref = Eigen::VectorXd::Constant(N, P.get("v_target"));
    r.acceleration_ref = Eigen::VectorXd::Zero(N);

    for (int i = 0; i < N; ++i) {
        r.X_ref(i) = (double)poly.path[i].x;
        r.Y_ref(i) = (double)poly.path[i].y;

        if (i < (int)poly.kappa.size()) r.curvature(i) = poly.kappa[i];
        else                            r.curvature(i) = 0.0;
    }

    r.valid = true;
    r.geo_eligible = true;
    r.curv_eligible = true;
    r.mpc_eligible = false;      // open path → ja zostawiam geo-only, tak jak miałeś
    r.all_path_eligible = false;
    r.yaw0 = yaw0;
    return r;
}

// =====================================================
// Polyline densify (linear interpolation)
// =====================================================
static inline void linear_interpolate_path(std::vector<Vec2>& path, const ParamBank& P)
{
    if (path.size() < 2) return;

    std::vector<Vec2> out;
    out.reserve(path.size() * 4);
    out.push_back(path[0]);

    const double ds = P.get("distance_between_interpoleted_points");
    if (ds <= 1e-9) return;

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Vec2 A = path[i];
        const Vec2 B = path[i+1];

        const Vec2 AB = B - A;
        const double seg_len = std::hypot(AB.x, AB.y);

        if (seg_len < 1e-12) {
            out.push_back(B);
            continue;
        }

        const int n_inside = (int)std::floor(seg_len / ds);
        Vec2 dir;
        dir.x = AB.x / (float)seg_len;
        dir.y = AB.y / (float)seg_len;

        for (int j = 1; j <= n_inside; ++j) {
            Vec2 p;
            p.x = A.x + dir.x * (float)(ds * j);
            p.y = A.y + dir.y * (float)(ds * j);
            out.push_back(p);
        }

        out.push_back(B);
    }

    path = std::move(out);
}

// =====================================================
// Sampling: open spline -> polyline with kappa
// =====================================================
static inline Poly_with_kappa sample_open_spline_to_poly_with_kappa(
    const TrackSpline2D& sp,
    double ds)
{
    Poly_with_kappa out;
    if (!sp.valid()) return out;

    const double L = sp.totalLength();
    if (L <= 1e-9) return out;

    const int maxN = 50000;
    int N = (int)std::ceil(L / ds) + 1;
    N = std::clamp(N, 2, maxN);

    out.path.reserve(N);
    out.kappa.reserve(N);

    double s = 0.0;
    for (int i = 0; i < N; ++i) {
        out.path.push_back(sp.eval(s));
        out.kappa.push_back(sp.getCurvature(s));

        s += ds;
        if (s > L) s = L;
    }

    return out;
}

// =====================================================
// Polyline modification along spline tangents
// =====================================================

static inline Vec2 unitTangentFromYaw(double yaw)
{
    return Vec2((float)std::cos(yaw), (float)std::sin(yaw));
}

static inline Vec2 tangentAtStart(const TrackSpline2D& sp)
{
    const double L = sp.totalLength();
    const double eps = std::min(0.05, 0.01 * std::max(1e-6, L));
    return unitTangentFromYaw(sp.getYaw(eps));
}

static inline Vec2 tangentAtEnd(const TrackSpline2D& sp)
{
    const double L = sp.totalLength();
    const double eps = std::min(0.05, 0.01 * std::max(1e-6, L));
    return unitTangentFromYaw(sp.getYaw(std::max(0.0, L - eps)));
}

static inline void add_extra_point_before_start_along_spline_tangent(
    Poly_with_kappa& poly,
    const TrackSpline2D& sp,
    double distance,const ParamBank& P)
{
    if (poly.path.size() < 2) return;

    Vec2 p0 = sp.eval(0.0);
    Vec2 t0 = tangentAtStart(sp); // jednostkowy wektor styczny na początku
    const double ds = P.get("distance_between_interpoleted_points");
    int number_of_points = (int)std::ceil(distance / ds);
    if (number_of_points < 1) number_of_points = 1;
    for(int i = 1; i <= number_of_points; ++i) {
        Vec2 newp;
        newp.x = p0.x - t0.x * (float)(ds * i);
        newp.y = p0.y - t0.y * (float)(ds * i);
        poly.path.insert(poly.path.begin(), newp);
        poly.kappa.insert(poly.kappa.begin(), 0.0); // dobudowane -> kappa=0
    }

}

static inline void add_extra_point_after_end_along_spline_tangent(
    Poly_with_kappa& poly,
    const TrackSpline2D& sp,
    double distance,
    const ParamBank& P)
{
    if (poly.path.size() < 2) return;

    const double L = sp.totalLength();
    Vec2 pL = sp.eval(L);
    Vec2 tL = tangentAtEnd(sp); // jednostkowy wektor styczny na końcu

    const double ds = P.get("distance_between_interpoleted_points");
    int number_of_points = (int)std::ceil(distance / ds);
    if (number_of_points < 1) number_of_points = 1;
    for(int i = 1; i <= number_of_points; ++i) {
        Vec2 newp;
        newp.x = pL.x + tL.x * (float)(ds * i);
        newp.y = pL.y + tL.y * (float)(ds * i);

        poly.path.push_back(newp);
        poly.kappa.push_back(0.0); // dobudowane -> kappa=0
    }

}


// =====================================================
// Straight line handler (2 points)
// =====================================================
static inline PathProcessResult path_procces_straight_line_2_points(
    const Eigen::VectorXd& X_path,
    const Eigen::VectorXd& Y_path,
    const State& bolide_state,
    const ParamBank& P)
{
    const double ds = P.get("distance_between_interpoleted_points");

    const double p0_x = X_path(0);
    const double p0_y = Y_path(0);
    const double p1_x = X_path(1);
    const double p1_y = Y_path(1);

    const double dx = p1_x - p0_x;
    const double dy = p1_y - p0_y;
    const double L  = std::hypot(dx, dy);

    if (L < ds) {
        return makeGeoInvalidResult();
    }

    const double t = find_t_on_segment(
        Vec2((float)p0_x, (float)p0_y),
        Vec2((float)p1_x, (float)p1_y),
        Vec2((float)bolide_state.X, (float)bolide_state.Y)
    );

    const double dist_to_p1 = -(t - 1.0) * L;

    double L_new = L;
    if (dist_to_p1 < P.get("min_path_length_for_geo") && t <= 1.0 && t > 0.0)
        L_new += P.get("min_path_length_for_geo") - dist_to_p1;
    if (t > 1.0)
        L_new += P.get("min_path_length_for_geo") - dist_to_p1;


    if(t >= 0.0) {
        
        const size_t pts_number = (size_t)std::ceil(L_new / ds) + 1;
        Eigen::VectorXd X_tmp((int)pts_number);
        Eigen::VectorXd Y_tmp((int)pts_number);

        const double bolid_x_proj = p0_x + t * dx;;
        const double bolid_y_proj = p0_y + t * dy;
    
        const double scale = L_new / L;
        for (size_t i = 0; i < pts_number; ++i) {
            const double u = (double)i / (double)(pts_number - 1);
            X_tmp((int)i) = bolid_x_proj + scale * u * dx;
            Y_tmp((int)i) = bolid_y_proj + scale * u * dy;
        }
    
        const double yaw0 = yawFrom2Pts(p0_x, p0_y, p1_x, p1_y);
        return makeCurvInvalidResult(X_tmp, Y_tmp, yaw0, P);

    }

    if(t < 0.0)
    {
        const double dist_to_p0 = -t * L;
        L_new += 0.5 + dist_to_p0;
        const size_t pts_number = (size_t)std::ceil(L_new / ds) + 1;

        Eigen::VectorXd X_tmp((int)pts_number);
        Eigen::VectorXd Y_tmp((int)pts_number);
    
        const double scale = L_new / L;
        for (size_t i = 0; i < pts_number; ++i) {
            const double u = (double)i / (double)(pts_number - 1);
            X_tmp((int)i) = p0_x + scale * u * dx;
            Y_tmp((int)i) = p0_y + scale * u * dy;
        }
        const double yaw0 = yawFrom2Pts(p0_x, p0_y, p1_x, p1_y);
        return makeCurvInvalidResult(X_tmp, Y_tmp, yaw0, P);
        
    }

  
}

// =====================================================
// Open spline handler (>=3 points)
// =====================================================
static inline PathProcessResult path_process_3_or_more_points_open_spline(
    const Eigen::VectorXd& X_path,
    const Eigen::VectorXd& Y_path,
    const State& bolide_state,
    const ParamBank& P)
{
    std::vector<Vec2> base = eigenToVec2(X_path, Y_path);

    TrackSpline2D sp;
    sp.build(base, false);

    const Vec2 q((float)bolide_state.X, (float)bolide_state.Y);

    // ---------- FALLBACK: spline invalid -> linear interpolate ----------
    if (!sp.valid()) {
        const int closest_seg = find_closest_segment_index(base, q);
        ROS_WARN_STREAM("[PathProcessing] Open spline build failed. Fallback to linear interpolation. Closest segment index: " << closest_seg);

        // przed początkiem
        if (closest_seg == 0 && base.size() >= 2) {
            const double t = find_t_on_segment(base[0], base[1], q);
            if (t < 0.0) {
                const double seg_len = std::hypot(base[1].x - base[0].x, base[1].y - base[0].y);
                const double dist_to_p0 = -t * seg_len;

                Vec2 dir = base[0] - base[1];
                const double dlen = std::hypot(dir.x, dir.y);
                if (dlen > 1e-12) {
                    dir.x /= (float)dlen;
                    dir.y /= (float)dlen;
                    Vec2 extra;
                    extra.x = base[0].x + dir.x * (float)(dist_to_p0 + 0.5);
                    extra.y = base[0].y + dir.y * (float)(dist_to_p0 + 0.5);
                    base.insert(base.begin(), extra);
                }
            }
        }

        // za końcem
        if (closest_seg == (int)base.size() - 2 && base.size() >= 2) {
            const int n = (int)base.size();
            const double t = find_t_on_segment(base[n-2], base[n-1], q);

            if (t > 1.0) {
                const double seg_len = std::hypot(base[n-1].x - base[n-2].x, base[n-1].y - base[n-2].y);
                const double dist_to_p1 = (t - 1.0) * seg_len;

                const double extra = P.get("min_path_length_for_geo") + dist_to_p1;
                if (extra > 0.0) {
                    Vec2 dir = base[n-1] - base[n-2];
                    const double dlen = std::hypot(dir.x, dir.y);
                    if (dlen > 1e-12) {
                        dir.x /= (float)dlen;
                        dir.y /= (float)dlen;
                        Vec2 extraP;
                        extraP.x = base[n-1].x + dir.x * (float)extra;
                        extraP.y = base[n-1].y + dir.y * (float)extra;
                        base.push_back(extraP);
                    }
                }
            }
        }

        linear_interpolate_path(base, P);

        const int ci = find_closest_segment_index(base, q);
        if (ci < 0 || ci + 1 >= (int)base.size()) return makeGeoInvalidResult(); // should not happen

        const double yaw0 = yawFrom2Pts(base[ci], base[ci + 1]);
        return makeCurvInvalidResult(base, yaw0, P);
    }

    // ---------- MAIN: spline valid -> sample -> closest on poly -> dobudowy ----------
    const double ds = P.get("distance_between_interpoleted_points");
    Poly_with_kappa poly = sample_open_spline_to_poly_with_kappa(sp, ds);
    if (poly.size() < 2) return makeGeoInvalidResult(); // should not happen
    
    int closest_i = find_closest_segment_index(poly.path, q);
    if (closest_i < 0 || closest_i + 1 >= (int)poly.size()) return makeGeoInvalidResult(); // should not happen
    // i==0: sprawdź czy auto przed początkiem
    if (closest_i == 0)
    {
        const double t = find_t_on_segment(poly.path[0], poly.path[1], q);
        if (t < 0.0) {
            const double seglen = std::hypot(poly.path[1].x - poly.path[0].x, poly.path[1].y - poly.path[0].y);
            const double dist_to_p0 = -t * seglen;
            add_extra_point_before_start_along_spline_tangent(poly, sp, dist_to_p0 + 0.5, P);
        }
    }

    // i==last: auto za końcem
    if (closest_i == (int)poly.size() - 2)
    {
        const int n = (int)poly.size();
        const double t = find_t_on_segment(poly.path[n-2], poly.path[n-1], q);
        if (t > 1.0) {
            const double seglen = std::hypot(poly.path[n-1].x - poly.path[n-2].x, poly.path[n-1].y - poly.path[n-2].y);
            const double dist_to_p1 = (t - 1.0) * seglen;
            const double extra = P.get("min_path_length_for_geo") + dist_to_p1;
            if (extra > 0.0) add_extra_point_after_end_along_spline_tangent(poly, sp, extra, P);
        }
    }

    // yaw0 bierzemy z polilinii po dobudowach
    closest_i = find_closest_segment_index(poly.path, q);
    if (closest_i < 0 || closest_i + 1 >= (int)poly.size()) return makeGeoInvalidResult(); // should not happen
    const double yaw0 = yawFrom2Pts(poly.path[closest_i], poly.path[closest_i + 1]);

    return makeResultFromPolyWithKappa(poly, yaw0, P);
}

// =====================================================
// Forward declaration: all-path velocity planner (closed loop)
// =====================================================
inline PathProcessResult all_path_and_velocity_planner_process_for_control(
    const ParamBank& P,
    const TrackSpline2D& spline_closed,
    const State& bolide_state);

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////   VELOCITY PLANNER (NIC NIE USUWAM)   ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

// =====================================================
// Velocity planner structs
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

// friction ellipse longitudinal availability
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
            v[i+1] = safeSqrt(v0i*v0i + 2.0*a[i]*ds);
            v[i+1] = std::min(v[i+1], v_sat_next);
            is_valid[i] = true;
        }
        else if (a_to_sat >= a_av_min && a_to_sat < a_av_max) {
            a[i] = a_to_sat;
            v[i+1] = v_sat_next;
            is_valid[i] = true;
        }
        else {
            a[i] = 0.0;
            v[i+1] = v_sat_next;
            is_valid[i] = false;
        }

        v[i+1] = std::clamp(v[i+1], v_min, v_max);
        v[i+1] = std::min(v[i+1], vsat[i+1]);
    }

    a[N-1] = a[N-2];
}

// backward: your idea
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

        const double v0_max  = safeSqrt(v1*v1 - 2.0*a_brake*ds);  
        v[i-1] = std::min(v[i-1], v0_max);  
        a[i-1] = (v1*v1 - v[i-1]*v[i-1]) / (2.0*ds);
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

        a[i] = ai;
        v[i+1] = v1;
        a_prev = ai;
    }

    a[N-1] = a[N-2];
}

// main planner: forward/backward + (jerk clamp -> FULL backward)
static inline SpeedProfileGeom forward_backward_pass_with_jerk_full_backward(
    const TrackSpline2D& sp_closed,
    double s0,
    double ds_geom,
    double S_plan,
    double v0_along,
    double v_min,
    double v_max,
    double a_acc_max,
    double a_dec_max,
    double a_lat_max,
    double jerk_up,
    double jerk_down,
    double a0_along,
    int merge_iter,
    const ParamBank& P)
{
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
            v, a, is_valid);

        fullBackwardPass_fix(
            prof.kappa, vsat,
            ds_geom,
            a_acc_max, a_dec_max, a_lat_max,
            v, a, is_valid);
    }

    const double smooth_jerk_up = jerk_up * 1.0/P.get("vel_planner_final_jerk_smooth_factor");
    const double smooth_jerk_down = jerk_down * 1.0/P.get("vel_planner_final_jerk_smooth_factor");
    final_jerkForwardClamp(
        prof.kappa, vsat,
        ds_geom,
        a_acc_max, a_dec_max, a_lat_max,
        smooth_jerk_up,smooth_jerk_down,
        a0_along,
        v, a, is_valid);

    // final clamps
    for (int i = 0; i < N; ++i) {
        double vi = v[i];
        if (!std::isfinite(vi)) vi = 0.0;
        vi = std::clamp(vi, v_min, v_max);
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

    auto lerp = [&](double x0, double x1) -> double {
        return (1.0 - alpha)*x0 + alpha*x1;
    };

    res.v_ref = lerp(prof.v[i],     prof.v[i+1]);
    res.a_ref = lerp(prof.a[i],     prof.a[i+1]);
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
    (void)ds_mpc; // zostawiam, bo u Ciebie było (na razie nie używasz)

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

    const double mpc_dt = 1.0 / P.get("odom_frequency");

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
        merge_iter,
        P
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
    for (int i = 0; i < K; ++i) {
        s_along = wrapS(s_along, L);

        const auto pr = profile_atDistance(prof, s_along);

        out.X_ref(i) = spline_closed.getX(s_along);
        out.Y_ref(i) = spline_closed.getY(s_along);
        out.curvature(i) = spline_closed.getCurvature(s_along);
        out.velocity_ref(i) = pr.v_ref;
        out.acceleration_ref(i) = pr.a_ref;

        s_along += pr.v_ref * mpc_dt + 0.5 * pr.a_ref * mpc_dt * mpc_dt;
    }

    return out;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////   END OF VELOCITY PLANNER BLOCK   ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

// =====================================================
// MAIN entry point
// =====================================================
inline PathProcessResult path_process_for_control(
    const ParamBank& P,
    const Eigen::VectorXd& X_path,
    const Eigen::VectorXd& Y_path,
    const State& bolide_state,
     const Vec2& prev_last, bool prev_last_valid = false,bool all_path_eligible_flag = false )
{
    if (X_path.size() != Y_path.size()) {
        ROS_WARN_STREAM("[PathProcessing] INVALID: X_path.size() != Y_path.size()");
        return PathProcessResult{};
    }

    // -------------------------------------------------
    // ALL PATH mode (zamknięta pętla + velocity planner)
    // -------------------------------------------------
    if (all_path_eligible_flag) {
        std::vector<Vec2> path_base = eigenToVec2(X_path, Y_path);

        TrackSpline2D spline_closed;
        spline_closed.build(path_base, true);

        if (!spline_closed.valid()) {
            ROS_WARN_STREAM("[PathProcessing] Closed spline build failed. Fallback to open spline.");

            if (X_path.size() >= 3) {
                return path_process_3_or_more_points_open_spline(X_path, Y_path, bolide_state, P);
            }
            if (X_path.size() == 2) {
                ROS_WARN_STREAM("[PathProcessing] Full path flag set, but got 2 points -> fallback to straight line.");
                return path_procces_straight_line_2_points(X_path, Y_path, bolide_state, P);
            }

            ROS_WARN_STREAM("[PathProcessing] Full path flag set, but got <2 points -> skipping update.");
            return makeGeoInvalidResult();
        }

        return all_path_and_velocity_planner_process_for_control(P, spline_closed, bolide_state);
    }

    // -------------------------------------------------
    // NORMAL mode (open path / local)
    // -------------------------------------------------
    const int Nraw = (int)X_path.size();
    if (Nraw == 0) {
        ROS_WARN_STREAM("[PathProcessing] INVALID: Nraw == 0");
        return PathProcessResult{};
    }

    // !!! Nraw == 1 zostawiam nietknięte !!!
    if (Nraw == 1) {

        if (prev_last_valid) {
    
            const double bx = bolide_state.X;
            const double by = bolide_state.Y;
    
            const double x_prev = (double)prev_last.x;
            const double y_prev = (double)prev_last.y;
    
            const double x_new  = (double)X_path(0);
            const double y_new  = (double)Y_path(0);
    
            const double d2_prev = (x_prev - bx)*(x_prev - bx) + (y_prev - by)*(y_prev - by);
            const double d2_new  = (x_new  - bx)*(x_new  - bx) + (y_new  - by)*(y_new  - by);
    
            // ------------------------------------------------------------
            // path_procces_straight_line_2_points() zakłada, że:
            // p0 = bliżej auta, p1 = dalej (czyli "do przodu" w sensie geometrii)
            // ------------------------------------------------------------
            Eigen::VectorXd X_single(2);
            Eigen::VectorXd Y_single(2);
    
            if (d2_prev <= d2_new) {
                // prev jest bliżej -> prev jako p0, nowy jako p1
                X_single(0) = x_prev;
                Y_single(0) = y_prev;
                X_single(1) = x_new;
                Y_single(1) = y_new;
            } else {
                // nowy jest bliżej -> nowy jako p0, prev jako p1
                X_single(0) = x_new;
                Y_single(0) = y_new;
                X_single(1) = x_prev;
                Y_single(1) = y_prev;
            }
    
            ROS_WARN_STREAM("[PathProcessing] Nraw == 1, prev point valid -> building 2-point line (sorted by distance to bolide).");
            return path_procces_straight_line_2_points(X_single, Y_single, bolide_state, P);
        }
    
        ROS_WARN_STREAM("[PathProcessing] INVALID: Nraw == 1 and no valid previous last point.");
        return PathProcessResult{};
    }

    if (Nraw == 2) {
        return path_procces_straight_line_2_points(X_path, Y_path, bolide_state, P);
    }

    // Nraw >= 3
    return path_process_3_or_more_points_open_spline(X_path, Y_path, bolide_state, P);
}

} // namespace v2_control
