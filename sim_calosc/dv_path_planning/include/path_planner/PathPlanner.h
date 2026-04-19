#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <set>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include "dv_interfaces/Cones.h"
#include "../rviz/RvizMarkers.h"
#include "../common/BolideDescriptor.h"
#include "cone_chain/ConeLookup.h"
#include "std_msgs/Float32MultiArray.h"

class PathPlanner
{
public:
    PathPlanner();
    virtual void plan(pcl::PointCloud<pcl::PointXYZL> &cones, BolideDescriptor &bolide) = 0;
    virtual void plan(const dv_interfaces::Cones &cones_msg, BolideDescriptor &bolide) = 0;

    virtual void post_PoseCallbackCallback(const ros::TimerEvent &event) {};

    /**
     * Publish rviz markers to /path_planning/markers topic.
     */
    virtual void publishRviz() = 0;

    std::vector<Vec2> getPath() const { return path; }

protected:
    BolideDescriptor bolide = BolideDescriptor(Vec2(), 0);;
    std::vector<Vec2> path; // Interpolated (if interpolation is applied)
    std::vector<Vec2> root_path; // Just the nodes that get fed into the interpolation
    std::vector<Vec2> track_true_center;
    ConeLookup cl;
    ros::Timer bolide_CoG_timer;
    bool closed_loop = false; // Planning algo flag that denotes whether the loop was closed during the planning

    /**
     * Obtain detected cones from slam.
     */
    void coneCallback(const sensor_msgs::PointCloud2::ConstPtr &cones_pcl);

    /**
     * Obtain detected cones from fake cone detector.
     */
    void fakeConeCallback(const dv_interfaces::Cones &cones_msg);

    /**
     * Obtain current position from odometry.
     */
    void poseCallback(const ros::TimerEvent &event);

    /**
     * Converts quaternions to angle in radians relative to x axis.
     */
    static double getYawFromQuaternion(const geometry_msgs::Quaternion msg);

    /**
     * Publish path points from path vector to /path_planning/path topic.
     */
    void publishPath(ros::Time ros_time);

    void trackCenterCallback(const std_msgs::Float32MultiArray &msg);

    void interpolateLinear(std::vector<Vec2>* path, int num_points);

    /**
     * Fits a polynomial function to the path points using Gauss-Newton optimization
     * @param path_imported Input points to fit the function through
     * @param dist_x Distance between interpolated points
     * @param max_points Maximum number of points to generate
     * @param max_iter Maximum optimization iterations
     * @param precision Convergence criterion
     * @return Vector of polynomial coefficients [a0, a1, a2, ...]
     */
    void interpolateGaussNewton(std::vector<Vec2>& path_imported, 
                                              double dist_x,
                                              int max_points,
                                              int polynomial_degree_input,
                                              int max_iter, 
                                              double precision);

    static void interpolateCubicSpline(std::vector<Vec2>& path, float dist_x, size_t max_points);
    static void interpolateAkimaSpline(std::vector<Vec2>& path, float dist_x, size_t max_points);

    /**
     *
     * @param path pre interpolation path
     * @param strength smoothing strength
     * @param low_pass_freq
     * @param general_strength
     * @param general_freq
     * @param cones_backtrack number of cones to use for "pre-loading" the derivative (0 for open tracks (discovery))
     */
    static void smoothPath(std::vector<Vec2> &path, float strength, float low_pass_freq, float general_strength, float general_freq, int
                           cones_backtrack = 4);

    static void smoothLowpassPreserveCorners(std::vector<Vec2> &path, float strength, int kernel_radius, float corner_low,
                                  float corner_high);

    struct SimpleOptimizerOptions {
        int grid_steps = 7;         // initial grid per-dimension
        int max_iters = 200;        // coordinate descent iterations
        double tol = 1e-5;          // stopping tolerance
        double shrink = 0.05;        // step shrink factor

        SimpleOptimizerOptions() {};
    };

    struct DEOptions {
        int population_size = 50;    // number of candidate solutions
        int max_generations = 200;   // iteration budget
        double F = 0.8;              // differential weight [0,2]
        double CR = 0.9;             // crossover probability [0,1]
        double tol = 1e-6;           // absolute tolerance for early stopping on objective
        int stagnation_generations = 40; // stop if no improvement for this many generations
        bool verbose = false;        // log progress
        unsigned int rng_seed = 0;   // 0 => auto-seed with time
        bool minimize = true;        // true=> minimize objective, false=> maximize
    };

    static double computePathError(const std::vector<Vec2> &generated, const std::vector<Vec2> &target,
                                   float sample_spacing, bool symmetric, float p, float max_weight,
                                   float max_weight_exp);

    static std::vector<double> simpleOptimize(const std::function<double(const std::vector<double>&)> &f,
                                              const std::vector<std::pair<double,double>> &bounds,
                                              const SimpleOptimizerOptions &opts = SimpleOptimizerOptions());

    static std::pair<std::vector<double>, double> differential_evolution(
        const std::function<double(const std::vector<double> &)> &obj,
        const std::vector<std::pair<double, double>> &bounds,
        const DEOptions &opts_in);

    /**
     * Evaluates the polynomial at given x
     * @param x X coordinate
     * @param coeffs Polynomial coefficients [a0, a1, a2, ...]
     */
    double evaluatePolynomial(double x, const std::vector<double>& coeffs);

    ros::NodeHandle nh = ros::NodeHandle("~");
    ros::Subscriber cone_sub;
    ros::Subscriber track_center_sub;
    ros::Publisher path_pub;
    RvizMarkers rvizMgr = RvizMarkers(nh);
};

#endif // PATH_PLANNER_H