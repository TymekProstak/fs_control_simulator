#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <vector>
#include <dv_interfaces/Path.h>

#include "../../include/path_planner/PathPlanner.h"
#include "../../include/common/Spline.h"
#include "../../include/common/config.h"

PathPlanner::PathPlanner()
{
    path.reserve(256);
    path_pub = nh.advertise<dv_interfaces::Path>("/path_planning/path", 10);
    
    bool useFakeConeDetector = false;
    if (!nh.getParam("fake_cone_detector", useFakeConeDetector))
        ROS_ERROR("[Path planning] Fake cone detector param loaded incorrectly!");

    if (useFakeConeDetector)
        cone_sub = nh.subscribe("/dv_cone_detector_fake/cones", 1, &PathPlanner::fakeConeCallback, this);
    else
        cone_sub = nh.subscribe("/slam/cones", 1, &PathPlanner::coneCallback, this);

    track_center_sub = nh.subscribe("/path_planning/track_true_center", 1, &PathPlanner::trackCenterCallback, this);

    ROS_INFO("[Path planning] bolide CoG timer initialized");
    // Asynchronous pose callback, to always have the latest info
    bolide_CoG_timer = nh.createTimer(ros::Duration(0.1), &PathPlanner::poseCallback, this);
}

void PathPlanner::coneCallback(const sensor_msgs::PointCloud2::ConstPtr &cones_pcl)
{
    ros::WallTime start = ros::WallTime::now();

    //poseCallback(ros::TimerEvent());
    pcl::PointCloud<pcl::PointXYZL>::Ptr cones(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::fromROSMsg(*cones_pcl, *cones);
    plan(*cones, bolide);
    ROS_INFO_THROTTLE(3, "[PathPlanning]: Planning algorithm duration: %.2f ms", (ros::WallTime::now() - start).toNSec() * 1e-6);

    publishPath(cones_pcl->header.stamp);
    publishRviz();
}

void PathPlanner::fakeConeCallback(const dv_interfaces::Cones &cones_msg)
{
    ros::WallTime start = ros::WallTime::now();
    //poseCallback(ros::TimerEvent()); // This slows down path planning by a constant amount (at least 200ms, at worst 5s)
    plan(cones_msg, bolide);
    ROS_INFO_THROTTLE(3, "[Fake PathPlanning]: Planning algorithm -> Duration: %.2f ms", (ros::WallTime::now() - start).toNSec() * 1e-6);

    publishPath(ros::Time::now());
    publishRviz();
}

void PathPlanner::poseCallback(const ros::TimerEvent &event)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    try
    {
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("map", "bolide_CoG", ros::Time(0), ros::Duration(0.2));

        float x = transformStamped.transform.translation.x;
        float y = transformStamped.transform.translation.y;
        float yaw = getYawFromQuaternion(transformStamped.transform.rotation);
        //ROS_INFO("[Path planning]: bolide pos: x: %f, y: %f", x, y);
        bolide.setCoordinates(Vec2(x, y), yaw);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_THROTTLE(30, "[Path Planning] Bolide pose transformation error: %s", ex.what()); // This might spam...
    }

    post_PoseCallbackCallback(event);
}

double PathPlanner::getYawFromQuaternion(const geometry_msgs::Quaternion msg)
{
    // More info about quaternions: https://www.youtube.com/watch?v=zjMuIxRvygQ
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    // More info about RPY: https://pl.wikipedia.org/wiki/K%C4%85ty_RPY
    // Yaw is the rotation around the z-axis
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

void PathPlanner::publishPath(ros::Time ros_time)
{
    dv_interfaces::Path msgPath;
    msgPath.full_path_enabled = closed_loop;
    geometry_msgs::PoseArray &poseArray = msgPath.path;
    poseArray.header.stamp = ros_time;
    poseArray.header.frame_id = "map";
    //interpolateLinear(path, NUMBER_OF_PATH_POINTS);

    for (const auto &point : path)
    {
        geometry_msgs::Pose pose;
        pose.position.x = point.x;
        pose.position.y = point.y;
        poseArray.poses.push_back(pose);
    }
    path_pub.publish(msgPath);
}

void PathPlanner::trackCenterCallback(const std_msgs::Float32MultiArray &msg) {
    // Convert the message data to a vector of Vec2s
    track_true_center.clear();
    for (int i = 0; i < msg.data.size(); i += 2) {
        track_true_center.emplace_back(msg.data[i], msg.data[i+1]);
    }
}

void PathPlanner::interpolateLinear(std::vector<Vec2>* path_imported, int num_points)
{
    std::vector<Vec2> &path = *path_imported;
    if (path.size() < 2)
        return;
    const int path_size = path.size();
    std::vector<Vec2> interpolated_path;
    interpolated_path.push_back(path[0]);

    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        Vec2 start = path[i];
        Vec2 end = path[i + 1];

        Vec2 direction = end - start;
        float segment_length = direction.length();
        direction /= segment_length; // Normalize

        int points_to_add = std::max(1, static_cast<int>( num_points / path_size - 1));

        for (int j = 1; j <= points_to_add; ++j)
        {
            float t = static_cast<float>(j) / points_to_add;
            Vec2 new_point = start + direction * (t * segment_length);
            interpolated_path.push_back(new_point);
        }
    }

    *path_imported = interpolated_path;
}

double PathPlanner::evaluatePolynomial(double x, const std::vector<double>& coeffs) {
    double result = 0.0;
    double x_power = 1.0;
    for (double coeff : coeffs) {
        result += coeff * x_power;
        x_power *= x;
    }
    return result;
}

/*******************************************************************************
 * Fits a polynomial function to the path points using Gauss-Newton optimization
 * @param path_imported Input points to fit the function through
 * @param dist_x Distance between interpolated points
 * @param max_points Maximum number of points to generate
 * @param max_iter Maximum optimization iterations
 * @param precision Convergence criterion
 *******************************************************************************/
void PathPlanner::interpolateGaussNewton(
    std::vector<Vec2>& path,
    double dist_x,
    int max_points,
    int polynomial_degree_input,
    int max_iter,
    double precision)
{
    if (path.size() < 3)
        return;

    const int polynomial_degree = std::min(polynomial_degree_input, static_cast<int>(path.size()) - 1);
    const int num_params = polynomial_degree + 1;

    std::vector<double> params(num_params, 0.0);
    params[0] = path[0].y;
    if (num_params > 1)
        params[1] = (path.back().y - path[0].y) / (path.back().x - path[0].x);

    std::vector<double> delta(num_params);
    std::vector<std::vector<double>> J(path.size(), std::vector<double>(num_params));
    std::vector<double> residuals(path.size());

    for (int iter = 0; iter < max_iter; ++iter) {
        double total_error = 0.0;

        for (size_t i = 0; i < path.size(); ++i) {
            double x = path[i].x;
            double predicted_y = evaluatePolynomial(x, params);
            residuals[i] = path[i].y - predicted_y;
            total_error += residuals[i] * residuals[i];

            double x_power = 1.0;
            for (int j = 0; j < num_params; ++j) {
                J[i][j] = -x_power;
                x_power *= x;
            }
        }

        if (total_error < precision)
            break;

        std::vector<std::vector<double>> JTJ(num_params, std::vector<double>(num_params, 0.0));
        std::vector<double> JTr(num_params, 0.0);

        for (int i = 0; i < num_params; ++i) {
            for (int j = 0; j < num_params; ++j) {
                for (size_t k = 0; k < path.size(); ++k)
                    JTJ[i][j] += J[k][i] * J[k][j];
            }
            for (size_t k = 0; k < path.size(); ++k)
                JTr[i] += J[k][i] * residuals[k];
        }

        for (int i = 0; i < num_params; ++i)
            JTJ[i][i] += 1e-6;

        // Gaussian elimination (forward elimination)
        for (int i = 0; i < num_params; ++i) {
            double diag = JTJ[i][i];
            for (int j = i; j < num_params; ++j)
                JTJ[i][j] /= diag;
            JTr[i] /= diag;
            for (int j = i + 1; j < num_params; ++j) {
                double factor = JTJ[j][i];
                for (int k = i; k < num_params; ++k)
                    JTJ[j][k] -= factor * JTJ[i][k];
                JTr[j] -= factor * JTr[i];
            }
        }

        // Back substitution
        for (int i = num_params - 1; i >= 0; --i) {
            delta[i] = JTr[i];
            for (int j = i + 1; j < num_params; ++j)
                delta[i] -= JTJ[i][j] * delta[j];
        }

        for (int i = 0; i < num_params; ++i)
            params[i] -= delta[i];
    //    cout << "Total error = " << total_error << endl;
    }

    // Interpolated points
    std::vector<Vec2> interpolated_path;
    double x_min = path.front().x;
    double x_max = path.back().x;
    for (int i = 0; i < max_points; ++i) {
        float x = x_min + i * dist_x;
        //if (x > x_max)
        //    break;
        float y = evaluatePolynomial(x, params);
        interpolated_path.push_back({ x, y });
    }

    path = interpolated_path;
}

// TODO
//  Add some low-pass filter to the base path, before interpolation (?)

// Used for both Cubic and Akima
struct SplineSegment {
    double a, b, c, d;
};

/// 1D Natural cubic spline solver: given parameter vector t (strictly increasing)
/// and samples y, returns n-1 segments for intervals [t[i], t[i+1]].
/// Removes degenerate intervals (zero step) should be avoided by caller.
static std::vector<SplineSegment> computeNaturalCubic(const std::vector<double>& t, const std::vector<double>& y) {
    const size_t n = t.size();
    std::vector<SplineSegment> segs;
    if (n < 2) return segs;
    segs.resize(n - 1);

    std::vector<double> h(n - 1);
    for (size_t i = 0; i < n - 1; ++i) h[i] = t[i + 1] - t[i];

    // Build alpha (only indices 1..n-2 used)
    std::vector<double> alpha(n, 0.0);
    for (size_t i = 1; i < n - 1; ++i) {
        if (h[i] == 0.0 || h[i - 1] == 0.0) {
            alpha[i] = 0.0;
            continue;
        }
        alpha[i] = 3.0 * ( (y[i+1] - y[i]) / h[i] - (y[i] - y[i-1]) / h[i-1] );
    }

    std::vector<double> l(n), mu(n), z(n), c(n), b(n - 1), d(n - 1);
    l[0] = 1.0;
    mu[0] = z[0] = 0.0;

    for (size_t i = 1; i < n - 1; ++i) {
        l[i] = 2.0 * (t[i + 1] - t[i - 1]) - h[i - 1] * mu[i - 1];
        // Guard against nearly-zero diagonal
        if (std::abs(l[i]) < 1e-12) {
            // fallback to small value to avoid division by zero
            l[i] = 1e-12;
        }
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0;
    z[n - 1] = c[n - 1] = 0.0;

    for (int j = int(n) - 2; j >= 0; --j) {
        c[j] = z[j] - mu[j] * c[j + 1];
        if (h[j] == 0.0) {
            b[j] = 0.0;
            d[j] = 0.0;
        } else {
            b[j] = (y[j + 1] - y[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }
        segs[j] = { y[j], b[j], c[j], d[j] };
    }

    return segs;
}

static inline Vec2 evaluateSegment(const SplineSegment& seg, double dt) {
    double v = seg.a + seg.b * dt + seg.c * dt * dt + seg.d * dt * dt * dt;
    return Vec2(float(v), 0.0f); // for 1D use only; caller will use for x and y separately
}

// Parametric cubic spline interpolation for 2D path.
// dist_x: desired spacing between output points (must be > 0).
// max_points: optional cap to avoid runaway memory usage (0 = no cap).
static void interpolateCubicSplineParametric(const std::vector<Vec2>& path,
                                             float dist_x,
                                             std::vector<Vec2>& interpolated_path,
                                             size_t max_points = 0)
{
    interpolated_path.clear();
    if (path.size() == 0) return;
    if (path.size() == 1) {
        interpolated_path.push_back(path.front());
        return;
    }
    if (dist_x <= 0.0f) {
        // invalid spacing -> return original path
        interpolated_path = path;
        return;
    }

    // 1) Remove consecutive duplicate points (very small distance)
    std::vector<Vec2> pts;
    pts.reserve(path.size());
    const double eps = 1e-9;
    for (size_t i = 0; i < path.size(); ++i) {
        if (i == 0) { pts.push_back(path[i]); continue; }
        double dx = double(path[i].x - path[i-1].x);
        double dy = double(path[i].y - path[i-1].y);
        if (dx*dx + dy*dy > eps) pts.push_back(path[i]);
    }
    if (pts.size() < 2) {
        // all points identical
        interpolated_path.push_back(pts.front());
        return;
    }

    // 2) Parameterize by cumulative chord length t
    const size_t n = pts.size();
    std::vector<double> t(n, 0.0);
    for (size_t i = 1; i < n; ++i) {
        double dx = double(pts[i].x - pts[i-1].x);
        double dy = double(pts[i].y - pts[i-1].y);
        t[i] = t[i-1] + std::sqrt(dx*dx + dy*dy);
    }

    // If total length is zero (all same), return single point
    double total_length = t.back();
    if (total_length <= 1e-12) {
        interpolated_path.push_back(pts.front());
        return;
    }

    // 3) Build 1D splines for x(t) and y(t)
    std::vector<double> xs(n), ys(n);
    for (size_t i = 0; i < n; ++i) { xs[i] = pts[i].x; ys[i] = pts[i].y; }

    auto splineX = computeNaturalCubic(t, xs);
    auto splineY = computeNaturalCubic(t, ys);
    if (splineX.empty() || splineY.empty()) {
        interpolated_path = pts;
        return;
    }

    // Helper to evaluate parametric spline at parameter T
    auto evalAt = [&](double T) -> Vec2 {
        // find interval i such that t[i] <= T <= t[i+1]
        size_t i = std::upper_bound(t.begin(), t.end(), T) - t.begin();
        if (i == 0) i = 1;
        if (i >= n) i = n - 1;
        --i;
        double dt = T - t[i];
        // evaluate 1D cubic at dt (segs are for interval i)
        const SplineSegment &sx = splineX[i];
        const SplineSegment &sy = splineY[i];
        double vx = sx.a + sx.b * dt + sx.c * dt * dt + sx.d * dt * dt * dt;
        double vy = sy.a + sy.b * dt + sy.c * dt * dt + sy.d * dt * dt * dt;
        return Vec2(float(vx), float(vy));
    };

    // 4) For each interval approximate segment length and produce samples
    interpolated_path.reserve(std::min<size_t>(size_t(total_length / dist_x) + 10, max_points==0?size_t(1e9):max_points));
    // push first control point
    interpolated_path.push_back(pts[0]);
    double accumulatedTotal = 0.0;
    for (size_t i = 0; i < n - 1; ++i) {
        double t0 = t[i];
        double t1 = t[i + 1];
        double segT = t1 - t0;
        if (segT <= 0.0) continue;

        // approximate arc length for interval [t0,t1] by subdividing
        int subdiv = 16; // tuned: enough for smooth approx; can be adjusted
        double segLength = 0.0;
        Vec2 prev = evalAt(t0);
        for (int s = 1; s <= subdiv; ++s) {
            double u = double(s) / double(subdiv);
            double T = t0 + u * segT;
            Vec2 cur = evalAt(T);
            double dx = double(cur.x - prev.x);
            double dy = double(cur.y - prev.y);
            segLength += std::sqrt(dx*dx + dy*dy);
            prev = cur;
        }

        if (segLength <= 1e-9) continue;

        int num_samples = std::max<int>(1, int(std::floor(segLength / double(dist_x))));
        // We will create `num_samples` new points uniformly in parameter T across this interval
        for (int s = 1; s <= num_samples; ++s) {
            double u = double(s) / double(num_samples + 1); // divide into (num_samples+1) gaps to not include t1 yet
            double T = t0 + u * segT;
            Vec2 p = evalAt(T);
            // avoid adding duplicates (tiny distances)
            Vec2 last = interpolated_path.back();
            double dx = double(p.x - last.x), dy = double(p.y - last.y);
            if (dx*dx + dy*dy > 1e-12) interpolated_path.push_back(p);
            if (max_points && interpolated_path.size() >= max_points) return;
        }

        // finally add end of interval (will be added again as start of next interval except for last)
        // but we ensure we don't duplicate the same point twice
        Vec2 endPt = pts[i + 1];
        Vec2 last = interpolated_path.back();
        double dx = double(endPt.x - last.x), dy = double(endPt.y - last.y);
        if (dx*dx + dy*dy > 1e-12) {
            interpolated_path.push_back(endPt);
            if (max_points && interpolated_path.size() >= max_points) return;
        }
    }
}

// TODO
//  Change Akima from double to float (?)

// --- 1D Akima helper: compute slopes and derivatives, then interval polynomials
// t : parameter (strictly increasing), size n
// y : samples at those t, size n
// returns n-1 segments (one per interval)
static std::vector<SplineSegment> computeAkima1D(const std::vector<float>& t, const std::vector<float>& y) {
    const size_t n = t.size();
    std::vector<SplineSegment> segs;
    if (n < 2) return segs;
    size_t m = n - 1; // number of intervals

    // compute interval slopes m_j for j=0..m-1
    std::vector<float> mvals(m);
    for (size_t j = 0; j < m; ++j) {
        float dt = t[j+1] - t[j];
        if (dt == 0.0) mvals[j] = 0.0;
        else mvals[j] = (y[j+1] - y[j]) / dt;
    }

    // extend mvals by 4 values (m_{-2}, m_{-1}, m_{m}, m_{m+1}) using linear extrapolation
    // We build m_ext such that m_ext[k] = m_{k-2}, so m_ext[2 + j] = mvals[j], length = m + 4
    std::vector<float> m_ext(m + 4);
    for (size_t j = 0; j < m; ++j) m_ext[2 + j] = mvals[j];
    // extrapolate inward
    m_ext[1] = 2.0 * m_ext[2] - m_ext[3];
    m_ext[0] = 2.0 * m_ext[1] - m_ext[2];
    // extrapolate outward
    m_ext[m + 2] = 2.0 * m_ext[m + 1] - m_ext[m];
    m_ext[m + 3] = 2.0 * m_ext[m + 2] - m_ext[m + 1];

    // compute derivatives d_i at data points i=0..n-1
    std::vector<float> d(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        // mapping: for point i, we want to use m_{i-2}, m_{i-1}, m_i, m_{i+1}
        size_t k = i + 2; // index into m_ext corresponding to m_i
        float w1 = std::abs(m_ext[k+1] - m_ext[k]);
        float w2 = std::abs(m_ext[k-1] - m_ext[k-2]);
        float denom = w1 + w2;
        if (denom > 0.0) {
            d[i] = (w1 * m_ext[k-1] + w2 * m_ext[k]) / denom;
        } else {
            // fallback to simple average
            d[i] = 0.5 * (m_ext[k-1] + m_ext[k]);
        }
    }

    // build interval cubic coefficients from Hermite form using y[i], y[i+1], d[i], d[i+1]
    segs.resize(m);
    for (size_t i = 0; i < m; ++i) {
        float h = t[i+1] - t[i];
        if (h == 0.0) {
            // degenerate interval: constant function
            segs[i] = { y[i], 0.0, 0.0, 0.0 };
            continue;
        }
        float y0 = y[i], y1 = y[i+1];
        float d0 = d[i], d1 = d[i+1];
        // polynomial in dt (dt in [0, h]):
        // a = y0
        // b = d0
        // c = (3*(y1 - y0)/h - 2*d0 - d1) / h
        // d = (2*(y0 - y1)/h + d0 + d1) / (h*h)
        float a = y0;
        float b = d0;
        float c = (3.0 * (y1 - y0) / h - 2.0 * d0 - d1) / h;
        float dd = (2.0 * (y0 - y1) / h + d0 + d1) / (h * h);
        segs[i] = { a, b, c, dd };
    }

    return segs;
}

// Evaluate a single 1D segment at dt
static inline float evalSeg1D(const SplineSegment& s, float dt) {
    return s.a + s.b * dt + s.c * dt * dt + s.d * dt * dt * dt;
}

// --- Parametric 2D Akima interpolation wrapper
// path: input points
// dist_x: desired approximate spacing (>0)
// interpolated_path: output vector (cleared and filled)
// max_points: optional cap (0 => no cap)
static void interpolateAkimaParametric(const std::vector<Vec2>& path,
                                       float dist_x,
                                       std::vector<Vec2>& interpolated_path,
                                       size_t max_points = 0)
{
    interpolated_path.clear();
    if (path.empty()) return;
    if (path.size() == 1) {
        interpolated_path.push_back(path.front());
        return;
    }
    if (dist_x <= 0.0f) {
        interpolated_path = path;
        return;
    }

    // 1) remove consecutive duplicate points
    std::vector<Vec2> pts;
    pts.reserve(path.size());
    const float eps = 1e-12;
    for (size_t i = 0; i < path.size(); ++i) {
        if (i == 0) { pts.push_back(path[i]); continue; }
        float dx = float(path[i].x - path[i-1].x);
        float dy = float(path[i].y - path[i-1].y);
        if (dx*dx + dy*dy > eps) pts.push_back(path[i]);
    }
    if (pts.size() < 2) {
        interpolated_path.push_back(pts.front());
        return;
    }

    // 2) chord-length parameterization t
    const size_t n = pts.size();
    std::vector<float> t(n, 0.0);
    for (size_t i = 1; i < n; ++i) {
        float dx = float(pts[i].x - pts[i-1].x);
        float dy = float(pts[i].y - pts[i-1].y);
        t[i] = t[i-1] + std::sqrt(dx*dx + dy*dy);
    }
    float totalLen = t.back();
    if (totalLen <= 1e-12) {
        interpolated_path.push_back(pts.front());
        return;
    }

    // x(t) and y(t) vectors
    std::vector<float> xs(n), ys(n);
    for (size_t i = 0; i < n; ++i) { xs[i] = pts[i].x; ys[i] = pts[i].y; }

    // 3) compute Akima 1D splines for x(t) and y(t)
    auto segsX = computeAkima1D(t, xs);
    auto segsY = computeAkima1D(t, ys);
    if (segsX.empty() || segsY.empty()) {
        // fallback to original points
        interpolated_path = pts;
        return;
    }

    // evaluator lambda for param T
    auto evalAt = [&](float T) -> Vec2 {
        // clamp T into [t0, t_{n-1}]
        if (T <= t.front()) return pts.front();
        if (T >= t.back()) return pts.back();
        // find interval index i: t[i] <= T < t[i+1]
        size_t i = std::upper_bound(t.begin(), t.end(), T) - t.begin();
        if (i == 0) i = 1;
        if (i >= n) i = n - 1;
        --i;
        float dt = T - t[i];
        float vx = evalSeg1D(segsX[i], dt);
        float vy = evalSeg1D(segsY[i], dt);
        return Vec2(float(vx), float(vy));
    };

    // 4) sample each interval approximately using arc-length subdivision (like before)
    interpolated_path.reserve(std::min<size_t>(size_t(totalLen / dist_x) + 10, max_points==0?size_t(1e9):max_points));
    interpolated_path.push_back(pts[0]);
    for (size_t i = 0; i < n - 1; ++i) {
        float t0 = t[i];
        float t1 = t[i+1];
        float segT = t1 - t0;
        if (segT <= 0.0) continue;

        // approximate arc length using subdivisions
        int subdiv = 4;
        float segLength = 0.0;
        Vec2 prev = evalAt(t0);
        for (int s = 1; s <= subdiv; ++s) {
            float u = float(s) / float(subdiv);
            float T = t0 + u * segT;
            Vec2 cur = evalAt(T);
            float dx = float(cur.x - prev.x);
            float dy = float(cur.y - prev.y);
            segLength += std::sqrt(dx*dx + dy*dy);
            prev = cur;
        }
        if (segLength <= 1e-12) continue;

        int num_samples = std::max<int>(1, int(std::floor(segLength / float(dist_x))));
        for (int s = 1; s <= num_samples; ++s) {
            float u = float(s) / float(num_samples + 1); // avoid duplicating interval endpoints
            float T = t0 + u * segT;
            Vec2 p = evalAt(T);
            Vec2 last = interpolated_path.back();
            float dx = float(p.x - last.x), dy = float(p.y - last.y);
            if (dx*dx + dy*dy > 1e-12) interpolated_path.push_back(p);
            if (max_points && interpolated_path.size() >= max_points) return;
        }

        // add interval's endpoint (avoid duplicates)
        Vec2 endPt = pts[i+1];
        Vec2 last = interpolated_path.back();
        float dx = float(endPt.x - last.x), dy = float(endPt.y - last.y);
        if (dx*dx + dy*dy > 1e-12) {
            interpolated_path.push_back(endPt);
            if (max_points && interpolated_path.size() >= max_points) return;
        }
    }
}

void PathPlanner::interpolateCubicSpline(std::vector<Vec2> &path, float dist_x, size_t max_points = 0) {
    std::vector<Vec2> out;
    interpolateCubicSplineParametric(path, dist_x, out, max_points);
    path.swap(out); // replace original path
}

void PathPlanner::interpolateAkimaSpline(std::vector<Vec2> &path, float dist_x, size_t max_points = 0) {
    std::vector<Vec2> out;
    interpolateAkimaParametric(path, dist_x, out, max_points); // Akima is just better
    path.swap(out); // replace original path
}

static inline float dot2(const Vec2 &a, const Vec2 &b) { return a.x*b.x + a.y*b.y; }
static inline float len2(const Vec2 &a) { return dot2(a,a); }
static inline float len(const Vec2 &a) { return std::sqrt(len2(a)); }
static inline Vec2 lerp(const Vec2 &a, const Vec2 &b, float t) { return a*(1.0f-t) + b*t; }

// curvature-aware gaussian low-pass smoothing
// - strength: [0..1] recommended, 0 = no smoothing, 1 = full smoothing
// - kernel_radius: use 1..10; typical 2..6. Larger -> heavier low-pass (more blurring).
// - corner_low/corner_high: angle thresholds (radians) which map angle -> how strongly we preserve corner.
//    angle <= corner_low -> no preservation (full smoothing); angle >= corner_high -> full preservation (no smoothing).
void PathPlanner::smoothLowpassPreserveCorners(std::vector<Vec2> &path,
                                               float strength = 0.5f,
                                               int kernel_radius = 3,
                                               float corner_low = 0.2f,   // ~11 degrees
                                               float corner_high = 1.0f)  // ~57 degrees
{
    // sanity checks and trivial cases
    if (strength <= 0.0f || path.size() < 3) return;
    if (kernel_radius < 1) kernel_radius = 1;
    size_t n = path.size();

    // Build Gaussian kernel (length = 2*R + 1)
    const int R = kernel_radius;
    const int K = 2*R + 1;
    std::vector<float> kernel(K);
    // sigma chosen so that kernel covers ~3*sigma within radius
    float sigma = std::max(0.5f, float(R) / 2.0f);
    float twoSigma2 = 2.0f * sigma * sigma;
    float sum = 0.0f;
    for (int i = -R; i <= R; ++i) {
        float v = std::exp(-(i*i) / twoSigma2);
        kernel[i + R] = v;
        sum += v;
    }
    // normalize
    for (int i = 0; i < K; ++i) kernel[i] /= sum;

    // Make a source copy so smoothing doesn't gradually cascade
    std::vector<Vec2> src = path;
    std::vector<Vec2> smooth(n);

    // Precompute smoothed position by convolving each coordinate with kernel
    for (size_t i = 0; i < n; ++i) {
        float sx = 0.0f, sy = 0.0f;
        for (int k = -R; k <= R; ++k) {
            // edge handling: clamp to ends (simple and robust)
            int idx = int(i) + k;
            if (idx < 0) idx = 0;
            if (idx >= int(n)) idx = int(n) - 1;
            sx += kernel[k + R] * src[idx].x;
            sy += kernel[k + R] * src[idx].y;
        }
        smooth[i] = Vec2(sx, sy);
    }

    // Helper: map angle -> preservation factor in [0,1]
    auto cornerPreserveFactor = [&](float ang) -> float {
        // ang in [0, pi]
        if (ang <= corner_low) return 0.0f;
        if (ang >= corner_high) return 1.0f;
        float t = (ang - corner_low) / (corner_high - corner_low); // 0..1
        // boost sharpness with ease curve (use smooth pow)
        return std::pow(t, 1.5f); // tweak exponent to taste (1.0 = linear)
    };

    // endpoints: keep exactly as original
    path[0] = src[0];
    path[n-1] = src[n-1];

    // For each interior point compute turning angle and combine original+smoothed accordingly
    for (size_t i = 1; i + 1 < n; ++i) {
        Vec2 A = src[i] - src[i-1];
        Vec2 B = src[i+1] - src[i];

        float lena = len(A);
        float lenb = len(B);

        float angle = 0.0f;
        if (lena > 1e-8f && lenb > 1e-8f) {
            // signed angle via atan2(cross,dot) but we only need absolute
            float cross = A.x * B.y - A.y * B.x;
            float dot = dot2(A, B);
            angle = std::fabs(std::atan2(cross, dot)); // in radians, range [0, pi]
        } else {
            // nearly degenerate; treat as no corner
            angle = 0.0f;
        }

        float preserve = cornerPreserveFactor(angle); // 0 => no preservation, 1 => full preservation
        float effective_strength = strength * (1.0f - preserve); // reduce smoothing at corners

        // final blended point
        // note: if effective_strength == 0 -> preserves original point exactly
        path[i] = lerp(src[i], smooth[i], effective_strength);
    }
}

// Smooth(er) first deriv.
/* First pass applies general track smoothing using a central derivative, then does 2 passes of smoothing to the derivative,
 * one going forwards, one going backwards. The first pass is meant to smooth track irregularities, while the last two
 * try to reconstruct the track on the corners in situations where the apex cone is not placed perfectly for the planning
 * algo to detect the apex. */
void PathPlanner::smoothPath(std::vector<Vec2> &path,
                             float strength,           // final smoothing strength for local passes
                             float low_pass_freq,      // alpha for low-pass (0..1)
                             float general_strength,   // blending for general smoothing (0..1)
                             float general_freq,       // alpha used when updating derivative accumulator (0..1)
                             int cones_backtrack)      // seed length from tail for accumulator
{
    // Quick guards
    if (strength == 0.0f || path.size() < 3) return;

    const size_t n = path.size();
    // If cones_backtrack is too large, clamp to at most (n-3) so we can safely compute centered derivatives
    int cb = std::max(0, cones_backtrack);
    cb = std::min(cb, static_cast<int>(n) - 3); // ensure at least indices [i-1,i,i+1] exist for i in [1..n-2]

    // helpers
    auto lerp = [](const Vec2 &a, const Vec2 &b, const float t) -> Vec2 {
        // a*(1-t) + b*t
        return a * (1.0f - t) + b * t;
    };

    auto centeredDerivative = [&](size_t i) -> Vec2 {
        // compute 0.5 * (forward_diff + backward_diff) = 0.5*( (p[i]-p[i-1]) + (p[i+1]-p[i]) )
        // which simplifies to 0.5*(p[i+1] - p[i-1])
        // valid only for 1 <= i <= n-2
        const Vec2 d = (path[i + 1] - path[i - 1]) * 0.5f;
        return d;
    };

    // ----- 1) General (anisotropic) smoothing pass (forward), seeded from tail if cb>0 -----
    Vec2 deriv_accum(0.0f, 0.0f);

    // seed deriv_accum: if cb>0, run a short propagation from index (startSeed) .. (n-2)
    // so the accumulator has knowledge of the tail before we run the full forward pass.
    if (cb > 0) {
        // choose start index such that we have cb interior points to walk across;
        // ensure start >= 1 and end <= n-2
        const size_t startSeed = static_cast<size_t>(std::max(1, static_cast<int>(n) - cb - 1));
        const size_t endSeed = n - 2; // inclusive

        // initialize deriv_accum with centered derivative at startSeed
        deriv_accum = centeredDerivative(startSeed);

        // propagate forward over seed window
        for (size_t i = startSeed; i <= endSeed; ++i) {
            Vec2 sample = centeredDerivative(i);
            deriv_accum = deriv_accum * general_freq + sample * (1.0f - general_freq);
        }
    } else {
        // default seed: use centered derivative at index 1
        deriv_accum = centeredDerivative(1);
    }

    // Now run the main forward general smoothing pass over interior points i = 1 .. n-2
    for (size_t i = 1; i <= n - 2; ++i) {
        const Vec2 prev = path[i - 1];
        // centered derivative at i
        const Vec2 sample = centeredDerivative(i);
        deriv_accum = deriv_accum * general_freq + sample * (1.0f - general_freq);
        const Vec2 proposed = prev + deriv_accum;                        // desired new location based on derivative
        path[i] = lerp(path[i], proposed, general_strength);        // blend original and proposed
    }

    // ----- 2) Make a copy for the backward/forward alternate pass -----
    std::vector<Vec2> back_path = path; // copy

    // ----- 3) Low-pass seeded pass that writes into path (tail-focused) -----
    // We'll process the tail region first if cb>0, then do full forward low-pass on back_path producing back_path result.

    // Preload deriv_accum for forward low-pass using the tail region if cb>0
    if (cb > 0) {
        // We'll use indices: startLP = n - cb - 1 .. n-2 for the seed (same logic as earlier but for low-pass)
        const size_t startLP = static_cast<size_t>(std::max(1, static_cast<int>(n) - cb - 1));
        const size_t endLP = n - 2;
        // initialize deriv_accum := forward diff at startLP (not centered here; trying to match original intent)
        // Use forward diff (p[i] - p[i-1]) for this seed phase to mimic original code's behavior
        deriv_accum = (back_path[startLP] - back_path[startLP - 1]);

        for (size_t i = startLP + 1; i <= endLP; ++i) {
            Vec2 forward_diff = back_path[i] - back_path[i - 1];
            deriv_accum = deriv_accum * low_pass_freq + forward_diff * (1.0f - low_pass_freq);
            // Apply update to original path at same index (as original code attempted)
            Vec2 proposed = back_path[i - 1] + deriv_accum;
            path[i] = lerp(path[i], proposed, strength);
        }
    } else {
        // default small seed
        deriv_accum = (back_path[1] - back_path[0]);
    }

    // Forward low-pass smoothing to compute back_path (we will use forward diffs)
    deriv_accum = (back_path[1] - back_path[0]); // reset to simple forward diff
    for (size_t i = 1; i < n; ++i) {
        const Vec2 forward_diff = back_path[i] - back_path[i - 1];
        deriv_accum = deriv_accum * low_pass_freq + forward_diff * (1.0f - low_pass_freq);
        back_path[i] = back_path[i - 1] + deriv_accum;
    }

    // ----- 4) Backward low-pass smoothing on path (process path in reverse) -----
    // Preload deriv_accum using head region (if cb > 0 we used it earlier; choose appropriate seed)
    if (cb > 0) {
        // seed with small forward-difference near the beginning
        deriv_accum = (path[1] - path[0]);
    } else {
        deriv_accum = (path[0] - path[1]); // original code used this fallback; keep sensible default
    }

    // Backwards loop: i = n-2 .. 0  (we skip last index because we need i+1 to exist)
    for (int si = static_cast<int>(n) - 2; si >= 0; --si) {
        size_t i = static_cast<size_t>(si);
        const Vec2 prev = path[i + 1]; // in backward sweep, "previous" refers to i+1
        const Vec2 cur = path[i];
        const Vec2 forward_diff = cur - prev;
        deriv_accum = deriv_accum * low_pass_freq + forward_diff * (1.0f - low_pass_freq);
        const Vec2 proposed = prev + deriv_accum;
        path[i] = lerp(path[i], proposed, strength);
    }

    // ----- 5) Final blend between the two passes -----
    for (size_t i = 0; i < n; ++i) {
        path[i] = (path[i] + back_path[i]) * 0.5f;
    }
}


static inline float clampf(float v, float a, float b) {
    return (v < a) ? a : (v > b) ? b : v;
}

// ==== Geometry helpers ====

// Closest point on segment AB to point P.
// Returns squared distance. If outClosest != nullptr, writes the closest point there.
static float pointSegmentDistSquared(const Vec2 &P, const Vec2 &A, const Vec2 &B, Vec2 *outClosest = nullptr) {
    // Handle degenerate segment
    Vec2 AB = B - A;
    float ab2 = AB.x*AB.x + AB.y*AB.y;
    if (ab2 <= 1e-12f) {
        // segment is a point
        if (outClosest) *outClosest = A;
        float dx = P.x - A.x, dy = P.y - A.y;
        return dx*dx + dy*dy;
    }
    // project AP onto AB, t in [0,1]
    Vec2 AP = P - A;
    float t = (AP.x * AB.x + AP.y * AB.y) / ab2;
    t = clampf(t, 0.0f, 1.0f);
    Vec2 C = A + AB * t;
    if (outClosest) *outClosest = C;
    float dx = P.x - C.x, dy = P.y - C.y;
    return dx*dx + dy*dy;
}

// Distance (squared) from point P to polyline (list of points interpreted as segment chain).
// If polyline has 0 points => large value. If 1 point => distance to that point.
static float pointToPolylineDistSquared(const Vec2 &P, const std::vector<Vec2> &poly) {
    if (poly.empty()) return std::numeric_limits<float>::infinity();
    if (poly.size() == 1) {
        float dx = P.x - poly[0].x, dy = P.y - poly[0].y;
        return dx*dx + dy*dy;
    }
    float best = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i + 1 < poly.size(); ++i) {
        float d2 = pointSegmentDistSquared(P, poly[i], poly[i+1], nullptr);
        if (d2 < best) best = d2;
    }
    return best;
}

// Resample polyline uniformly by arc length, spacing <= sample_spacing.
// If input has less than 2 points, returns the input.
static std::vector<Vec2> resamplePolylineUniform(const std::vector<Vec2> &poly, float sample_spacing) {
    std::vector<Vec2> out;
    if (poly.empty()) return out;
    if (poly.size() == 1) { out.push_back(poly[0]); return out; }
    if (sample_spacing <= 1e-6f) {
        // if spacing invalid, just return original
        return poly;
    }

    // compute segment lengths
    std::vector<float> segLen;
    segLen.reserve(poly.size()-1);
    float totalLen = 0.0f;
    for (size_t i = 0; i + 1 < poly.size(); ++i) {
        float dx = poly[i+1].x - poly[i].x;
        float dy = poly[i+1].y - poly[i].y;
        float L = std::sqrt(dx*dx + dy*dy);
        segLen.push_back(L);
        totalLen += L;
    }
    if (totalLen <= 1e-12f) { // all same
        out.push_back(poly.front());
        return out;
    }

    // target number of samples (include endpoints)
    size_t nSamples = 1 + (size_t)std::max( (float)0.0f, std::floor(totalLen / sample_spacing) );
    if (nSamples < 2) nSamples = 2;
    float actualSpacing = totalLen / float(nSamples - 1);

    out.reserve(nSamples);
    // walk along polyline creating samples at positions 0, actualSpacing, 2*actualSpacing ... totalLen
    float curTarget = 0.0f;
    size_t segIndex = 0;
    float segStartPos = 0.0f; // cumulative position at seg start
    out.push_back(poly.front());
    for (size_t s = 1; s < nSamples - 1; ++s) {
        curTarget = actualSpacing * float(s);
        // advance segIndex until curTarget inside segment
        while (segIndex < segLen.size() && segStartPos + segLen[segIndex] < curTarget) {
            segStartPos += segLen[segIndex];
            ++segIndex;
        }
        if (segIndex >= segLen.size()) {
            // numerical safety
            out.push_back(poly.back());
            continue;
        }
        float segT = (curTarget - segStartPos) / std::max(1e-12f, segLen[segIndex]);
        // clamp segT
        segT = clampf(segT, 0.0f, 1.0f);
        Vec2 A = poly[segIndex];
        Vec2 B = poly[segIndex + 1];
        Vec2 sample = A * (1.0f - segT) + B * segT;
        out.push_back(sample);
    }
    out.push_back(poly.back());
    return out;
}

// ==== Error metrics ====

// Parameters:
// - generated: produced path polyline
// - target: true center polyline
// - sample_spacing: spacing to resample target (<=0 means no resampling)
// - symmetric: if true, avg of target->generated and generated->target
// - p: Lp norm exponent (>=1). p=2 -> RMSE. p>2 penalizes large deviations more.
// - max_weight: if >0, adds (max_dist^max_weight_exp) term scaled and added to the error.
// - max_weight_exp: exponent applied to the max-distance term (>=1).
//
// Returns a non-negative error (double). Lower is better.
double PathPlanner::computePathError(const std::vector<Vec2> &generated,
                                     const std::vector<Vec2> &target,
                                     float sample_spacing = 0.25f,
                                     bool symmetric = false,
                                     float p = 2.0f,
                                     float max_weight = 0.0f,
                                     float max_weight_exp = 2.0f)
{
    // Basic validation
    if (generated.empty() || target.empty()) return std::numeric_limits<double>::infinity();
    if (p < 1.0f) p = 1.0f;
    if (max_weight < 0.0f) max_weight = 0.0f;
    if (max_weight_exp < 1.0f) max_weight_exp = 1.0f;

    // 1) resample target
    const std::vector<Vec2>& sampledTarget = target;
    // if (target.size() >= 2 && sample_spacing > 1e-6f) sampledTarget = resamplePolylineUniform(target, sample_spacing);
    // else sampledTarget = target;

    // helper: compute one-way Lp mean^ (1/p)
    auto one_way = [&](const std::vector<Vec2> &from,
                       const std::vector<Vec2> &to) -> double
    {
        if (from.empty() || to.empty()) return std::numeric_limits<double>::infinity();
        double acc = 0.0;
        double maxd = 0.0;
        for (const Vec2 &pnt : from) {
            double d2 = pointToPolylineDistSquared(pnt, to);
            double d = std::sqrt(d2);
            maxd = std::max(maxd, d);
            acc += std::pow(d, p);
        }
        double mean = acc / double(from.size());
        double lp = std::pow(mean, 1.0 / double(p));
        // add optional max penalty
        if (max_weight > 0.0) {
            double max_pen = std::pow(maxd, max_weight_exp);
            lp += max_weight * max_pen;
        }
        return lp;
    };

    double e_tg = one_way(sampledTarget, generated);
    if (!symmetric) return e_tg;

    // symmetric: also resample generated and compute generated -> sampledTarget
    std::vector<Vec2> sampledGen;
    if (generated.size() >= 2 && sample_spacing > 1e-6f) sampledGen = resamplePolylineUniform(generated, sample_spacing);
    else sampledGen = generated;

    double e_gt = one_way(sampledGen, sampledTarget);
    return 0.5 * (e_tg + e_gt);
}

// ==== Small dependency-free optimizer ====
// A simple coarse grid + local coordinate descent that takes a function
// f: vector<double> -> double (error) and tries to minimize it.
// This is intentionally simple, deterministic, and requires no external libraries.
// Useful for tuning a few parameters (<= ~6). For many parameters use a proper optimizer (NLopt etc.)

// bounds: for each parameter, (min,max)
std::vector<double> PathPlanner::simpleOptimize(const std::function<double(const std::vector<double>&)> &f,
                                          const std::vector<std::pair<double,double>> &bounds,
                                          const SimpleOptimizerOptions &opts)
{
    size_t dim = bounds.size();
    assert(dim > 0);

    // initial guess: center of bounds
    std::vector<double> x(dim);
    for (size_t i = 0; i < dim; ++i) x[i] = 0.5 * (bounds[i].first + bounds[i].second);

    // initial step sizes: fraction of range
    std::vector<double> step(dim);
    for (size_t i = 0; i < dim; ++i) step[i] = 0.25 * (bounds[i].second - bounds[i].first);

    double fx = f(x);
    for (int iter = 0; iter < opts.max_iters; ++iter) {
        printf("iter %d:\n", iter);
        for (int i = 0; i < dim; ++i) {
            printf("x[%d] = %.3f, ", i, x[i]);
        }
        printf("error: %.5f\n", fx);
        bool improved = false;
        for (size_t i = 0; i < dim; ++i) {
            // try +/- directions on coordinate i
            std::vector<double> x_plus = x;
            x_plus[i] = clampf(x_plus[i] + step[i], bounds[i].first, bounds[i].second);
            double fplus = f(x_plus);

            std::vector<double> x_minus = x;
            x_minus[i] = clampf(x_minus[i] - step[i], bounds[i].first, bounds[i].second);
            double fminus = f(x_minus);

            if (fplus < fx && fplus <= fminus) {
                x = x_plus; fx = fplus; improved = true;
            } else if (fminus < fx && fminus < fplus) {
                x = x_minus; fx = fminus; improved = true;
            }
        }
        if (!improved) {
            // shrink steps
            bool allSmall = true;
            for (size_t i = 0; i < dim; ++i) {
                step[i] *= opts.shrink;
                if (step[i] > opts.tol) allSmall = false;
            }
            if (allSmall) break;
        }
    }
    return x;
}

#include <random>

// Return type: pair(best_params, best_value)
std::pair<std::vector<double>, double>
PathPlanner::differential_evolution(const std::function<double(const std::vector<double>&)>& obj,
                       const std::vector<std::pair<double,double>>& bounds,
                       const DEOptions& opts_in = DEOptions())
{
    // Basic checks
    size_t dim = bounds.size();
    if (dim == 0) return {{}, 0.0};

    // Options (copy so we can modify)
    DEOptions opts = opts_in;

    // RNG
    std::mt19937_64 rng;
    if (opts.rng_seed == 0) {
        rng.seed((unsigned long long) std::chrono::high_resolution_clock::now().time_since_epoch().count());
    } else {
        rng.seed(opts.rng_seed);
    }
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    // Population size heuristic if not provided
    int NP = opts.population_size;
    if (NP <= 0) {
        NP = int(10 * dim); if (NP < 20) NP = 20;
    }

    // helper to sample uniformly within bounds
    auto sample_uniform_in_bounds = [&](std::vector<double>& x) {
        x.resize(dim);
        for (size_t i = 0; i < dim; ++i) {
            std::uniform_real_distribution<double> d(bounds[i].first, bounds[i].second);
            x[i] = d(rng);
        }
    };

    // initialize population and evaluate
    std::vector<std::vector<double>> pop(NP, std::vector<double>(dim));
    std::vector<double> pop_val(NP, std::numeric_limits<double>::infinity());
    for (int i = 0; i < NP; ++i) {
        sample_uniform_in_bounds(pop[i]);
        pop_val[i] = obj(pop[i]);
    }

    // find best
    int best_idx = 0;
    for (int i = 1; i < NP; ++i) {
        if (opts.minimize ? (pop_val[i] < pop_val[best_idx]) : (pop_val[i] > pop_val[best_idx])) {
            best_idx = i;
        }
    }
    double best_val = pop_val[best_idx];
    std::vector<double> best_vec = pop[best_idx];

    int stagn = 0;
    // main generational loop
    for (int gen = 0; gen < opts.max_generations; ++gen) {
        // for each target vector
        for (int i = 0; i < NP; ++i) {
            // pick three distinct indices a,b,c != i
            int a, b, c;
            do { a = int(uni01(rng) * NP); } while (a == i);
            do { b = int(uni01(rng) * NP); } while (b == i || b == a);
            do { c = int(uni01(rng) * NP); } while (c == i || c == a || c == b);

            const std::vector<double>& A = pop[a];
            const std::vector<double>& B = pop[b];
            const std::vector<double>& C = pop[c];

            std::vector<double> trial(dim);

            // mutation: V = A + F * (B - C)
            for (size_t k = 0; k < dim; ++k) {
                trial[k] = A[k] + opts.F * (B[k] - C[k]);
                // enforce bounds by clipping
                if (trial[k] < bounds[k].first) trial[k] = bounds[k].first + uni01(rng) * (bounds[k].second - bounds[k].first);
                if (trial[k] > bounds[k].second) trial[k] = bounds[k].first + uni01(rng) * (bounds[k].second - bounds[k].first);
            }

            // crossover: binomial
            std::vector<double> child(dim);
            std::uniform_int_distribution<int> dist_k(0, int(dim)-1);
            int jrand = dist_k(rng); // ensure at least one from mutant
            for (size_t k = 0; k < dim; ++k) {
                if (uni01(rng) < opts.CR || int(k) == jrand) child[k] = trial[k];
                else child[k] = pop[i][k];
            }

            // evaluate child
            double child_val = obj(child);

            // selection
            bool better = opts.minimize ? (child_val < pop_val[i]) : (child_val > pop_val[i]);
            if (better) {
                pop[i] = std::move(child);
                pop_val[i] = child_val;
                if (opts.minimize ? (child_val < best_val) : (child_val > best_val)) {
                    best_val = child_val;
                    best_vec = pop[i];
                    if (opts.verbose) {
                        std::cout << "gen " << gen << " new best val: " << best_val << "\n";
                    }
                    stagn = 0;
                }
            }
        }

        // generation done -> track stagnation
        stagn++;
        if (opts.verbose && gen % 10 == 0) {
            std::cout << "DE gen " << gen << " best=" << best_val << " stagn=" << stagn << "\n";
        }

        // stopping conditions
        if ((opts.minimize && best_val <= opts.tol) || (!opts.minimize && best_val >= opts.tol)) {
            if (opts.verbose) std::cout << "DE: tol reached at gen " << gen << "\n";
            break;
        }
        if (stagn >= opts.stagnation_generations) {
            if (opts.verbose) std::cout << "DE: stagnation stop at gen " << gen << "\n";
            break;
        }
    }

    return {best_vec, best_val};
}
