#pragma once

#include "Utils.h"

#include <ros/ros.h>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <deque>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>
#include <g2o/core/robust_kernel_impl.h>

class GraphManager
{
public:
    explicit GraphManager(ros::NodeHandle& nh);
    ~GraphManager();

    GraphManager(const GraphManager&) = delete;
    GraphManager& operator=(const GraphManager&) = delete;

    void updateOdom(const Pose2D& pose, const ros::Time& stamp);

    void addPoseAndObservations(const std::vector<Landmark>& observations, const Pose2D& pose_at_capture);

    bool lookupPose(const ros::Time& stamp, Pose2D& out) const;

    std::vector<Pose2D> getAllPoses() const;
    std::vector<Landmark> getAllLandmarks() const;
    std::vector<Landmark> getConfirmedLandmarks() const;

    Pose2D getMapToOdomOffset() { std::lock_guard<std::mutex> lk(mutex_); return map_to_odom_offset_; }
    
    // dupa
    std::atomic<bool> optimized_ = false;
    
private:

    void optimizerLoop();
    
    // Internal landmark creation (caller must already hold mutex_)
    int addLandmarkLocked(const Landmark& lm);

    int findNearestLandmarkKDTree(const Eigen::Vector2d& pt, const std::string& cone_type);
    int findNearestLandmarkLinearLocked(const Eigen::Vector2d& pt, const std::string& cone_type) const;
    void rebuildKDTree();
    double computeDistance(const Eigen::Vector2d& pt1, const Landmark& lm) const;
    
    /// Compute the 2x2 information matrix for an observation in local frame
    /// using the proper range-bearing noise model with Jacobian transformation.
    Eigen::Matrix2d computeObservationInformation(const Eigen::Vector2d& local_obs) const;

    /// Compute 3x3 odometry information matrix with optional adaptive
    /// downweighting for large odometry jumps.
    Eigen::Matrix3d computeOdometryInformation(double dx, double dy, double dtheta) const;

    // Merging landmarks that end up close to each other after optimization
    bool areLandmarksMergeCompatible(const Landmark& a, const Landmark& b) const;
    void fuseLandmarksInPlace(Landmark& keep, const Landmark& removed) const;
    void mergeNearbyLandmarks(
        std::unordered_map<int, Landmark>& landmarks,
        std::vector<int>& to_remove,
        std::unordered_map<int, int>& merged_into) const;

    // Current offset in reference to INS data
    Pose2D map_to_odom_offset_;
    
    // Odometry edge structure
    struct OdomEdge {
        int from_id;
        int to_id;
        Eigen::Vector3d measurement; // (dx, dy, dtheta)
        Eigen::Matrix3d information;
    };
    
    // Observation edge structure
    struct ObservationEdge {
        int pose_id;
        int landmark_id;
        Eigen::Vector2d measurement; // (x, y) in camera frame
        Eigen::Matrix2d information;
    };
    
    std::unordered_map<int, Landmark> landmarks_;
    std::unordered_map<int, Pose2D> poses_;
    
    // --- Configuration parameters (set once in constructor) ---
    double data_association_radius_;
    bool match_cone_types_;
    bool use_mahalanobis_;
    double mahalanobis_threshold_;
    int observation_count_threshold_;     // observations needed to become confirmed landmark
    double observation_min_range_;
    double observation_max_range_;
    double observation_max_fov_angle_;    // radians, symmetric ±angle
    double observation_range_noise_;      // meters
    double observation_bearing_noise_;    // radians
    double odometry_translation_noise_;   // meters
    double odometry_rotation_noise_;      // radians
    bool odometry_smoothing_enabled_;     // smooth incoming INS/RTK odom before SLAM usage
    double odometry_smoothing_alpha_;     // EMA alpha in [0,1], higher = less smoothing
    double odometry_robust_kernel_delta_; // Huber delta for 3DOF odom residual
    bool odometry_adaptive_info_enabled_; // downweight suspicious odom deltas via information matrix
    double odometry_translation_gate_;    // [m] translation delta above this is downweighted
    double odometry_rotation_gate_;       // [rad] rotation delta above this is downweighted
    double odometry_hard_gate_ratio_;     // stronger attenuation past this ratio
    double odometry_min_info_scale_;      // lower bound for information scaling
    bool offset_smoothing_enabled_;       // smooth map->odom offset updates from optimizer
    double offset_smoothing_alpha_;       // EMA alpha in [0,1] for map->odom offset
    int optimizer_iterations_;            // g2o iterations
    double optimizer_loop_rate_;          // Hz
    int min_poses_to_optimize_;           // minimum poses before optimizing
    double min_pose_distance_;            // minimum translation to add a new pose
    double min_pose_angle_;               // minimum rotation to add a new pose
    int max_observation_edges_;           // fixed sliding window size for graph size
    int max_odometry_edges_;
    IdGenerator id_gen_;
    
    // KD-tree for efficient spatial search
    pcl::PointCloud<pcl::PointXY>::Ptr landmark_cloud_;
    pcl::KdTreeFLANN<pcl::PointXY> kdtree_;
    std::vector<int> landmark_ids_; // Maps cloud index to landmark ID
    bool kdtree_needs_rebuild_;
    
    // g2o optimizer
    std::unique_ptr<g2o::SparseOptimizer> optimizer_;
    std::vector<OdomEdge> odom_edges_;
    std::vector<ObservationEdge> obs_edges_;
    
    // Timestamped odom buffer for pose lookup by cone message timestamp
    struct StampedPose {
        ros::Time stamp;
        Pose2D pose;
    };
    std::deque<StampedPose> raw_odom_buffer_;
    static constexpr size_t kMaxOdomBufferSize = 500; // ~10s at 50Hz

    // Absolute pose tracking (INS provides absolute positions)
    Pose2D first_pose_;
    Pose2D previous_pose_;       // Previous graph pose (with ID, for odom edge linking)
    Pose2D previous_raw_pose_;   // Raw odom at time of last added graph pose
    Pose2D previous_smoothed_odom_; // Last smoothed odometry sample
    bool has_smoothed_odom_ = false;
    bool has_map_to_odom_offset_ = false;
    bool first_pose_set_;
    bool has_raw_pose_ = false;
    int current_pose_id_ = -1;  // Track the most recent pose ID

    mutable std::mutex mutex_;
    std::thread optimizer_thread_;
    std::atomic<bool> running_{false};
};
