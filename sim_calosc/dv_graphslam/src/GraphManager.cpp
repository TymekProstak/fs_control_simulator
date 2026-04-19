#include "GraphManager.h"
#include "Utils.h"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <unordered_set>

GraphManager::GraphManager(ros::NodeHandle& nh)
{
    // Load data association parameters
    nh.param<double>("data_association/radius", data_association_radius_, 0.8);
    nh.param<bool>("data_association/match_cone_types", match_cone_types_, true);
    nh.param<bool>("data_association/use_mahalanobis", use_mahalanobis_, false);
    nh.param<double>("data_association/mahalanobis_threshold", mahalanobis_threshold_, 5.99);
    nh.param<int>("data_association/observation_count_threshold", observation_count_threshold_, 3);
    
    // Load observation gating parameters
    nh.param<double>("observation/min_range", observation_min_range_, 0.5);
    nh.param<double>("observation/max_range", observation_max_range_, 15.0);
    nh.param<double>("observation/range_noise", observation_range_noise_, 0.1);
    nh.param<double>("observation/bearing_noise", observation_bearing_noise_, 0.05);
    double max_fov_deg;
    nh.param<double>("observation/max_fov_angle", max_fov_deg, 180.0);
    observation_max_fov_angle_ = max_fov_deg * M_PI / 180.0; // Convert to radians
    
    // Load odometry noise parameters
    nh.param<double>("odometry/translation_noise", odometry_translation_noise_, 0.05);
    nh.param<double>("odometry/rotation_noise", odometry_rotation_noise_, 0.02);
    nh.param<bool>("odometry/smoothing_enabled", odometry_smoothing_enabled_, true);
    nh.param<double>("odometry/smoothing_alpha", odometry_smoothing_alpha_, 0.35);
    nh.param<double>("odometry/robust_kernel_delta", odometry_robust_kernel_delta_, std::sqrt(7.815));
    nh.param<bool>("odometry/adaptive_info_enabled", odometry_adaptive_info_enabled_, true);
    nh.param<double>("odometry/translation_gate", odometry_translation_gate_, 0.8);
    nh.param<double>("odometry/rotation_gate", odometry_rotation_gate_, 0.35);
    nh.param<double>("odometry/hard_gate_ratio", odometry_hard_gate_ratio_, 2.0);
    nh.param<double>("odometry/min_info_scale", odometry_min_info_scale_, 0.02);
    nh.param<bool>("odometry/offset_smoothing_enabled", offset_smoothing_enabled_, true);
    nh.param<double>("odometry/offset_smoothing_alpha", offset_smoothing_alpha_, 0.20);
    odometry_smoothing_alpha_ = std::clamp(odometry_smoothing_alpha_, 0.0, 1.0);
    offset_smoothing_alpha_ = std::clamp(offset_smoothing_alpha_, 0.0, 1.0);
    odometry_translation_gate_ = std::max(odometry_translation_gate_, 1e-3);
    odometry_rotation_gate_ = std::max(odometry_rotation_gate_, 1e-3);
    odometry_hard_gate_ratio_ = std::max(odometry_hard_gate_ratio_, 1.0);
    odometry_min_info_scale_ = std::clamp(odometry_min_info_scale_, 1e-6, 1.0);
    
    // Load optimizer parameters
    nh.param<int>("optimizer/iterations", optimizer_iterations_, 10);
    nh.param<double>("optimizer/loop_rate", optimizer_loop_rate_, 1.0);
    nh.param<int>("optimizer/min_poses_to_optimize", min_poses_to_optimize_, 5);
    nh.param<int>("optimizer/max_observation_edges", max_observation_edges_, 5000);
    nh.param<int>("optimizer/max_odometry_edges", max_odometry_edges_, 5000);
    
    // Minimum motion thresholds
    nh.param<double>("optimizer/min_pose_distance", min_pose_distance_, 0.3);
    double min_pose_angle_deg;
    nh.param<double>("optimizer/min_pose_angle", min_pose_angle_deg, 5.0);
    min_pose_angle_ = min_pose_angle_deg * M_PI / 180.0;
    
    // Initialize KD-tree
    landmark_cloud_ = pcl::PointCloud<pcl::PointXY>::Ptr(new pcl::PointCloud<pcl::PointXY>);
    kdtree_needs_rebuild_ = false;
    
    // Initialize g2o optimizer
    optimizer_ = std::make_unique<g2o::SparseOptimizer>();
    
    // Set up solver
    auto linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
    auto blockSolver = std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    optimizer_->setAlgorithm(algorithm);
    
    // Initialize pose tracking
    first_pose_set_ = false;
    
    ROS_INFO("[GraphManager] Data association radius: %.2f m", data_association_radius_);
    ROS_INFO("[GraphManager] Match cone types: %s", match_cone_types_ ? "true" : "false");
    ROS_INFO("[GraphManager] Observation count threshold: %d", observation_count_threshold_);
    ROS_INFO("[GraphManager] Use Mahalanobis distance: %s", use_mahalanobis_ ? "true" : "false");
    if (use_mahalanobis_) {
        ROS_INFO("[GraphManager] Mahalanobis threshold: %.2f", mahalanobis_threshold_);
    }
    ROS_INFO("[GraphManager] Observation range: [%.2f, %.2f] m", observation_min_range_, observation_max_range_);
    ROS_INFO("[GraphManager] Observation FOV: +/-%.1f deg", max_fov_deg);
    ROS_INFO("[GraphManager] Observation noise: range=%.3f m, bearing=%.3f rad", 
             observation_range_noise_, observation_bearing_noise_);
    ROS_INFO("[GraphManager] Odometry noise: translation=%.3f m, rotation=%.3f rad", 
             odometry_translation_noise_, odometry_rotation_noise_);
    ROS_INFO("[GraphManager] Odometry smoothing: enabled=%s, alpha=%.2f",
             odometry_smoothing_enabled_ ? "true" : "false", odometry_smoothing_alpha_);
    ROS_INFO("[GraphManager] Odometry adaptive info: enabled=%s, gate_t=%.2f m, gate_r=%.2f rad, hard_ratio=%.2f, min_scale=%.3f",
             odometry_adaptive_info_enabled_ ? "true" : "false",
             odometry_translation_gate_, odometry_rotation_gate_,
             odometry_hard_gate_ratio_, odometry_min_info_scale_);
    ROS_INFO("[GraphManager] map->odom offset smoothing: enabled=%s, alpha=%.2f",
             offset_smoothing_enabled_ ? "true" : "false", offset_smoothing_alpha_);
    ROS_INFO("[GraphManager] Optimizer: iterations=%d, loop_rate=%.1f Hz, min_poses=%d", 
             optimizer_iterations_, optimizer_loop_rate_, min_poses_to_optimize_);
    ROS_INFO("[GraphManager] Min motion: distance=%.3f m, angle=%.1f deg",
             min_pose_distance_, min_pose_angle_ * 180.0 / M_PI);
    
    running_.store(true);
    optimizer_thread_ = std::thread(&GraphManager::optimizerLoop, this);
    ROS_INFO("[GraphManager] Backend optimizer thread started");
}

GraphManager::~GraphManager()
{
    running_.store(false);
    
    if (optimizer_thread_.joinable()) {
        optimizer_thread_.join();
    }
    ROS_INFO("[GraphManager] Backend optimizer thread stopped");
}

void GraphManager::updateOdom(const Pose2D& pose, const ros::Time& stamp)
{
    std::lock_guard<std::mutex> lk(mutex_);

    Pose2D pose_to_store = pose;
    if (odometry_smoothing_enabled_) {
        if (!has_smoothed_odom_) {
            previous_smoothed_odom_ = pose;
            has_smoothed_odom_ = true;
        } else {
            const double alpha = odometry_smoothing_alpha_;

            pose_to_store.x = previous_smoothed_odom_.x + alpha * (pose.x - previous_smoothed_odom_.x);
            pose_to_store.y = previous_smoothed_odom_.y + alpha * (pose.y - previous_smoothed_odom_.y);

            const double dtheta = normalizeAngle(pose.theta - previous_smoothed_odom_.theta);
            pose_to_store.theta = normalizeAngle(previous_smoothed_odom_.theta + alpha * dtheta);
            pose_to_store.id = -1;

            previous_smoothed_odom_ = pose_to_store;
        }
    }

    raw_odom_buffer_.push_back({stamp, pose_to_store});
    if (raw_odom_buffer_.size() > kMaxOdomBufferSize) {
        raw_odom_buffer_.pop_front();
    }
    has_raw_pose_ = true;
}

bool GraphManager::lookupPose(const ros::Time& stamp, Pose2D& out) const
{
    std::lock_guard<std::mutex> lk(mutex_);
    if (raw_odom_buffer_.empty()) return false;

    // Binary search for the first element with stamp >= target
    auto it = std::lower_bound(
        raw_odom_buffer_.begin(), raw_odom_buffer_.end(), stamp,
        [](const StampedPose& sp, const ros::Time& t) { return sp.stamp < t; });

    if (it == raw_odom_buffer_.end()) {
        // All stamps are before target — use the latest
        out = raw_odom_buffer_.back().pose;
    } else if (it == raw_odom_buffer_.begin()) {
        // Target is at or before the oldest entry
        out = it->pose;
    } else {
        // INTERPOLACJA LINIOWA
        const auto& p1 = *it;           // Klatka "po"
        const auto& p0 = *std::prev(it); // Klatka "przed"

        // Obliczamy współczynnik czasu t (0.0 do 1.0)
        double duration_between = (p1.stamp - p0.stamp).toSec();
        double t = (stamp - p0.stamp).toSec() / duration_between;

        // Interpolacja X i Y
        out.x = p0.pose.x + t * (p1.pose.x - p0.pose.x);
        out.y = p0.pose.y + t * (p1.pose.y - p0.pose.y);

        // Interpolacja kąta Theta (z uwzględnieniem zawijania się kątów [-pi, pi])
        double delta_theta = normalizeAngle(p1.pose.theta - p0.pose.theta);
        out.theta = normalizeAngle(p0.pose.theta + t * delta_theta);
        
        out.id = -1; // Poza interpolowana nie ma własnego ID w grafie
    }
    return true;
}

int GraphManager::addLandmarkLocked(const Landmark& lm)
{
    // NOTE: Caller must already hold mutex_
    const int id = id_gen_.nextLandmarkId();
    Landmark l = lm;
    l.id = id;
    l.observation_count = std::max(1, l.observation_count);
    landmarks_.emplace(id, std::move(l));
    kdtree_needs_rebuild_ = true;
    return id;
}

int GraphManager::findNearestLandmarkLinearLocked(const Eigen::Vector2d& pt, const std::string& cone_type) const
{
    double best_d2 = data_association_radius_ * data_association_radius_;
    int best_id = -1;
    for (const auto& kv : landmarks_) {
        const Landmark& lm = kv.second;
        
        // Skip if cone types don't match (unless one is unknown)
        if (match_cone_types_ && 
            cone_type != "unknown" && 
            lm.type != "unknown" && 
            cone_type != lm.type) {
            continue;
        }
        
        const double dx = lm.mean.x() - pt.x();
        const double dy = lm.mean.y() - pt.y();
        const double d2 = dx*dx + dy*dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_id = lm.id;
        }
    }
    return best_id;
}

void GraphManager::rebuildKDTree()
{
    landmark_cloud_->clear();
    landmark_ids_.clear();
    
    for (const auto& kv : landmarks_) {
        pcl::PointXY pt;
        pt.x = kv.second.mean.x();
        pt.y = kv.second.mean.y();
        landmark_cloud_->push_back(pt);
        landmark_ids_.push_back(kv.first);
    }
    
    if (!landmark_cloud_->empty()) {
        kdtree_.setInputCloud(landmark_cloud_);
    }
    
    kdtree_needs_rebuild_ = false;
}

double GraphManager::computeDistance(const Eigen::Vector2d& pt, const Landmark& lm) const
{
    const Eigen::Vector2d diff = pt - lm.mean;
    if (use_mahalanobis_ && lm.cov.determinant() > 1e-10) {
        // Mahalanobis distance: d = sqrt( (x - mu)^T * Sigma^{-1} * (x - mu) )
        return std::sqrt(diff.transpose() * lm.cov.inverse() * diff);
    } else {
        return diff.norm();
    }
}

Eigen::Matrix2d GraphManager::computeObservationInformation(const Eigen::Vector2d& local_obs) const
{
    const double range = local_obs.norm();
    const double bearing = std::atan2(local_obs.y(), local_obs.x());
    
    const double cos_b = std::cos(bearing);
    const double sin_b = std::sin(bearing);
    
    // Jacobian of the polar-to-Cartesian transformation:
    //   x = r * cos(theta),  y = r * sin(theta)
    //   J = d(x,y)/d(r,theta) = [[cos(theta), -r*sin(theta)],
    //                              [sin(theta),  r*cos(theta)]]
    Eigen::Matrix2d J;
    J(0, 0) =  cos_b;
    J(0, 1) = -range * sin_b;
    J(1, 0) =  sin_b;
    J(1, 1) =  range * cos_b;
    
    // Covariance in polar frame: Sigma_polar = diag(sigma_range^2, sigma_bearing^2)
    Eigen::Matrix2d cov_polar = Eigen::Matrix2d::Zero();
    cov_polar(0, 0) = observation_range_noise_ * observation_range_noise_;
    cov_polar(1, 1) = observation_bearing_noise_ * observation_bearing_noise_;
    
    // Transform to Cartesian frame: Sigma_xy = J * Sigma_polar * J^T
    const Eigen::Matrix2d cov_cartesian = J * cov_polar * J.transpose();
    
    // Information matrix = Sigma_xy^{-1}
    // For a 2x2 SPD matrix this is always well-conditioned when range > 0
    return cov_cartesian.inverse();
}

Eigen::Matrix3d GraphManager::computeOdometryInformation(double dx, double dy, double dtheta) const
{
    Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
    information(0, 0) = 1.0 / (odometry_translation_noise_ * odometry_translation_noise_);
    information(1, 1) = 1.0 / (odometry_translation_noise_ * odometry_translation_noise_);
    information(2, 2) = 1.0 / (odometry_rotation_noise_ * odometry_rotation_noise_);

    if (!odometry_adaptive_info_enabled_) {
        return information;
    }

    const double trans_delta = std::sqrt(dx * dx + dy * dy);
    const double rot_delta = std::abs(dtheta);

    const double trans_ratio = trans_delta / odometry_translation_gate_;
    const double rot_ratio = rot_delta / odometry_rotation_gate_;

    auto ratioToScale = [this](double ratio) {
        if (ratio <= 1.0) {
            return 1.0;
        }

        double scale = 1.0 / (ratio * ratio);
        if (ratio > odometry_hard_gate_ratio_) {
            scale *= 0.25;
        }
        return std::max(odometry_min_info_scale_, scale);
    };

    const double trans_scale = ratioToScale(trans_ratio);
    const double rot_scale = ratioToScale(rot_ratio);

    information(0, 0) *= trans_scale;
    information(1, 1) *= trans_scale;
    information(2, 2) *= rot_scale;

    if (trans_ratio > odometry_hard_gate_ratio_ || rot_ratio > odometry_hard_gate_ratio_) {
        ROS_WARN_THROTTLE(1.0,
            "[GraphManager] Odom jump gated by information scaling: trans=%.2fm (x%.2f), rot=%.2frad (x%.2f), scales(t=%.3f,r=%.3f)",
            trans_delta, trans_ratio, rot_delta, rot_ratio, trans_scale, rot_scale);
    }

    return information;
}

bool GraphManager::areLandmarksMergeCompatible(const Landmark& a, const Landmark& b) const
{
    if (match_cone_types_ && a.type != "unknown" && b.type != "unknown" && a.type != b.type) {
        return false;
    }

    const Eigen::Vector2d diff = a.mean - b.mean;
    const double radius_sq = data_association_radius_ * data_association_radius_;

    // Fast pre-gate in Euclidean space.
    if (diff.squaredNorm() > radius_sq) {
        return false;
    }

    if (!use_mahalanobis_) {
        return true;
    }

    constexpr double kCovEps = 1e-6;
    Eigen::Matrix2d cov_sum = a.cov + b.cov;
    cov_sum(0, 0) += kCovEps;
    cov_sum(1, 1) += kCovEps;

    const double det = cov_sum.determinant();
    if (!std::isfinite(det) || det <= 1e-12) {
        return false;
    }

    const double d2 = diff.transpose() * cov_sum.inverse() * diff;
    return std::isfinite(d2) && d2 < mahalanobis_threshold_;
}

void GraphManager::fuseLandmarksInPlace(Landmark& keep, const Landmark& removed) const
{
    constexpr double kCovEps = 1e-6;

    Eigen::Matrix2d cov_keep = keep.cov;
    Eigen::Matrix2d cov_removed = removed.cov;
    cov_keep(0, 0) += kCovEps;
    cov_keep(1, 1) += kCovEps;
    cov_removed(0, 0) += kCovEps;
    cov_removed(1, 1) += kCovEps;

    const double det_keep = cov_keep.determinant();
    const double det_removed = cov_removed.determinant();
    const bool valid_keep = std::isfinite(det_keep) && det_keep > 1e-12;
    const bool valid_removed = std::isfinite(det_removed) && det_removed > 1e-12;

    // Information-form fusion when both covariances are valid.
    if (valid_keep && valid_removed) {
        const Eigen::Matrix2d info_keep = cov_keep.inverse();
        const Eigen::Matrix2d info_removed = cov_removed.inverse();
        const Eigen::Matrix2d info_sum = info_keep + info_removed;
        const double info_det = info_sum.determinant();

        if (std::isfinite(info_det) && info_det > 1e-12) {
            const Eigen::Matrix2d fused_cov = info_sum.inverse();
            const Eigen::Vector2d fused_mean = fused_cov *
                (info_keep * keep.mean + info_removed * removed.mean);
            keep.mean = fused_mean;
            keep.cov = fused_cov;
        }
    }

    keep.observation_count += removed.observation_count;

    // Keep known class if possible.
    if (keep.type == "unknown" && removed.type != "unknown") {
        keep.type = removed.type;
    }
}

void GraphManager::mergeNearbyLandmarks(
    std::unordered_map<int, Landmark>& landmarks,
    std::vector<int>& to_remove,
    std::unordered_map<int, int>& merged_into) const
{
    std::unordered_set<int> to_remove_set;

    for (auto it1 = landmarks.begin(); it1 != landmarks.end(); ++it1) {
        if (to_remove_set.find(it1->first) != to_remove_set.end()) {
            continue; // Already marked for removal
        }

        for (auto it2 = std::next(it1); it2 != landmarks.end(); ++it2) {
            if (to_remove_set.find(it2->first) != to_remove_set.end()) {
                continue; // Already marked for removal
            }

            if (!areLandmarksMergeCompatible(it1->second, it2->second)) {
                continue;
            }

            // Keep the one with more observations.
            if (it1->second.observation_count >= it2->second.observation_count) {
                fuseLandmarksInPlace(it1->second, it2->second);
                to_remove.push_back(it2->first);
                to_remove_set.insert(it2->first);
                merged_into[it2->first] = it1->first;
            } else {
                fuseLandmarksInPlace(it2->second, it1->second);
                to_remove.push_back(it1->first);
                to_remove_set.insert(it1->first);
                merged_into[it1->first] = it2->first;
                break; // it1 is marked for removal, move to next
            }
        }
    }
}

int GraphManager::findNearestLandmarkKDTree(const Eigen::Vector2d& pt, const std::string& cone_type)
{
    if (landmarks_.empty()) {
        return -1;
    }
    
    // Rebuild KD-tree if needed
    if (kdtree_needs_rebuild_) {
        rebuildKDTree();
    }
    
    // Safety check: if KD-tree has no input cloud, return -1
    if (!kdtree_.getInputCloud() || landmark_cloud_->empty()) {
        return -1;
    }
    
    // Search for nearest neighbors within radius
    pcl::PointXY search_point;
    search_point.x = static_cast<float>(pt.x());
    search_point.y = static_cast<float>(pt.y());
    
    std::vector<int> indices;
    std::vector<float> distances;
    
    // Search within radius
    const int found = kdtree_.radiusSearch(search_point, 
        static_cast<float>(data_association_radius_), indices, distances);
    
    if (found == 0) {
        return -1;
    }
    
    // Filter by cone type and find best match
    int best_id = -1;
    double best_distance = data_association_radius_;
    
    for (size_t i = 0; i < indices.size(); ++i) {
        const int landmark_id = landmark_ids_[indices[i]];
        const Landmark& lm = landmarks_.at(landmark_id);
        
        // Skip if cone types don't match (unless one is unknown)
        if (match_cone_types_ && 
            cone_type != "unknown" && 
            lm.type != "unknown" && 
            cone_type != lm.type) {
            continue;
        }
        
        const double dist = computeDistance(pt, lm);
        if (dist < best_distance) {
            best_distance = dist;
            best_id = landmark_id;
        }
    }
    
    return best_id;
}


void GraphManager::addPoseAndObservations(const std::vector<Landmark>& observations, const Pose2D& pose_at_capture_raw)
{
    std::lock_guard<std::mutex> lk(mutex_);

    if (!has_raw_pose_) {
        ROS_WARN("[GraphManager] No odometry received yet, skipping");
        return;
    }

    // --- Step 1: Create a new pose node from the pose at capture time ---

    // Get corrected pose (raw is from pure INS, this is corrected by offset calculated in SLAM)
    Pose2D pose_at_capture_corrected = applyOffset(pose_at_capture_raw, map_to_odom_offset_);

    bool should_add_new_pose = false;

    // Check minimum motion threshold (skip if robot hasn't moved enough)
    if (first_pose_set_) {
        const double dx_check = pose_at_capture_corrected.x - previous_pose_.x;
        const double dy_check = pose_at_capture_corrected.y - previous_pose_.y;
        const double dist = std::sqrt(dx_check * dx_check + dy_check * dy_check);
        const double dtheta_check = std::abs(normalizeAngle(pose_at_capture_corrected.theta - previous_pose_.theta));
        
        // add new pose only with sufficient movement
        if (dist >= min_pose_distance_ || dtheta_check >= min_pose_angle_) {
            
            should_add_new_pose = true;
        }
    } else {
        should_add_new_pose = true;
    }

    // temp
    const double sin_p = std::sin(pose_at_capture_corrected.theta);
    const double cos_p = std::cos(pose_at_capture_corrected.theta);

    if (!should_add_new_pose) {  
        // Still process observations to update landmark counts, but don't add to graph
        for (const auto& observation : observations) {
            const Eigen::Vector2d local(observation.mean.x(), observation.mean.y());
            const Eigen::Vector2d global = Eigen::Vector2d(
                pose_at_capture_corrected.x + local.x() * cos_p - local.y() * sin_p,
                pose_at_capture_corrected.y + local.x() * sin_p + local.y() * cos_p
            );

            int match_id = findNearestLandmarkKDTree(global, observation.type);
            if (match_id >= 0) {
                Landmark& lm = landmarks_.at(match_id);
                lm.observation_count++;
            }
        }
        return;
    }

    current_pose_id_ = id_gen_.nextPoseId();

    pose_at_capture_corrected.id = current_pose_id_;

    if (!first_pose_set_) {
        // First pose: initial estimate = raw odom
        Pose2D p;
        p.id =  current_pose_id_;
        p.x = pose_at_capture_raw.x;
        p.y = pose_at_capture_raw.y;
        p.theta = pose_at_capture_raw.theta;
        first_pose_ = p;
        first_pose_set_ = true;
        ROS_INFO("[GraphManager] First pose set as anchor at (%.2f, %.2f, %.2f)",
                p.x, p.y, p.theta);
    } else {
        // Compute odom delta from previous RAW pose
        const double cos_prev = std::cos(previous_raw_pose_.theta);
        const double sin_prev = std::sin(previous_raw_pose_.theta);
        const double dx_w = pose_at_capture_raw.x - previous_raw_pose_.x;
        const double dy_w = pose_at_capture_raw.y - previous_raw_pose_.y;

        // Transform to local point of reference of previous pose
        const double dx = dx_w * cos_prev + dy_w * sin_prev;
        const double dy = -dx_w * sin_prev + dy_w * cos_prev;
        const double dtheta = normalizeAngle(pose_at_capture_raw.theta - previous_raw_pose_.theta);
        

        // Create odometry edge
        OdomEdge odom_edge;
        odom_edge.from_id = previous_pose_.id;
        odom_edge.to_id = current_pose_id_;
        odom_edge.measurement = Eigen::Vector3d(dx, dy, dtheta);
        odom_edge.information = computeOdometryInformation(dx, dy, dtheta);

        odom_edges_.push_back(odom_edge);
    }

    previous_raw_pose_ = pose_at_capture_raw;
    previous_pose_ = pose_at_capture_corrected;
    poses_.emplace(current_pose_id_, pose_at_capture_corrected);

    
    // --- Step 2: Process observations ---

    // Previous pose frame (newly added keyframe)
    // const double sin_p = std::sin(previous_pose_.theta);
    // const double cos_p = std::cos(previous_pose_.theta);

    // Rotation world -> local(previous_pose) = R(theta)^T
    Eigen::Matrix2d R_w2l;
    R_w2l <<  cos_p, sin_p,
             -sin_p, cos_p;
    
    bool any_new_landmarks = false;
    
    for (const auto& observation : observations) {
        
        // Transform observation from local frame to global frame
        const Eigen::Vector2d local(observation.mean.x(), observation.mean.y());

        const Eigen::Vector2d global = Eigen::Vector2d(
            pose_at_capture_corrected.x + local.x() * cos_p - local.y() * sin_p,
            pose_at_capture_corrected.y + local.x() * sin_p + local.y() * cos_p
        );

        int match_id = findNearestLandmarkKDTree(global, observation.type);
        if (match_id >= 0) {
            Landmark& lm = landmarks_.at(match_id);


            lm.observation_count += observation.observation_count;
            if (observation.type != "unknown") lm.type = observation.type;
            
            ObservationEdge obs_edge;
            obs_edge.pose_id = previous_pose_.id;
            obs_edge.landmark_id = match_id;
            obs_edge.measurement = local;
            obs_edge.information = computeObservationInformation(local);
            obs_edges_.push_back(obs_edge);

        } else {

            // New landmark, pretty much same logic
            Landmark newlm = observation;
            newlm.mean = global;
            newlm.cov = observation.cov;
            newlm.observation_count = observation.observation_count;
            const int lm_id = addLandmarkLocked(newlm);
            any_new_landmarks = true;
            
            ObservationEdge obs_edge;
            obs_edge.pose_id = previous_pose_.id;
            obs_edge.landmark_id = lm_id;
            obs_edge.measurement = local;
            obs_edge.information = computeObservationInformation(local);
            obs_edges_.push_back(obs_edge);
        }
    }

    if(obs_edges_.size() > max_observation_edges_) {
        obs_edges_.erase(obs_edges_.begin(), obs_edges_.end() - max_observation_edges_);
    }

    if(odom_edges_.size() > max_odometry_edges_) {
        odom_edges_.erase(odom_edges_.begin(), odom_edges_.end() - max_odometry_edges_);
    }
    
    if (any_new_landmarks) {
        rebuildKDTree();
    }
}

std::vector<Pose2D> GraphManager::getAllPoses() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<Pose2D> out;
    out.reserve(poses_.size());
    for (const auto& kv : poses_) {
        out.push_back(kv.second);
    }
    return out;
}

std::vector<Landmark> GraphManager::getAllLandmarks() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<Landmark> out;
    out.reserve(landmarks_.size());
    for (const auto& kv : landmarks_) {
        out.push_back(kv.second);
    }
    return out;
}

std::vector<Landmark> GraphManager::getConfirmedLandmarks() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<Landmark> out;
    for (const auto& kv : landmarks_) {
        if (kv.second.observation_count >= observation_count_threshold_) {
            out.push_back(kv.second);
        }
    }
    return out;
}

void GraphManager::optimizerLoop()
{
    ROS_INFO("[GraphManager] Optimizer loop starting");
    
    // Compute loop period from configurable rate
    const int loop_period_ms = (optimizer_loop_rate_ > 0.0) 
        ? static_cast<int>(1000.0 / optimizer_loop_rate_) 
        : 1000;
    const auto loop_period = std::chrono::milliseconds(loop_period_ms);
    
    while (running_.load()) {
        std::this_thread::sleep_for(loop_period);
        
        if (!running_.load()) {
            break; // Exit immediately if stopped during sleep
        }
        
        // --- Phase 1: Copy data under lock ---
        std::unordered_map<int, Pose2D> poses_copy;
        std::unordered_map<int, Landmark> landmarks_copy;
        std::vector<OdomEdge> odom_edges_copy;
        std::vector<ObservationEdge> obs_edges_copy;
        Pose2D first_pose_copy;
        Pose2D previous_raw_pose_copy;
        bool first_pose_set_copy;
        int current_pose_id_copy = -1;
        
        {
            std::lock_guard<std::mutex> lk(mutex_);
            poses_copy = poses_;
            landmarks_copy = landmarks_;
            odom_edges_copy = odom_edges_;
            obs_edges_copy = obs_edges_;
            first_pose_copy = first_pose_;
            previous_raw_pose_copy = previous_raw_pose_;
            first_pose_set_copy = first_pose_set_;
            current_pose_id_copy = current_pose_id_;
        }
        // --- Lock released: callbacks are no longer blocked ---
        
        const size_t num_poses = poses_copy.size();
        const size_t num_landmarks = landmarks_copy.size();
        
        if (num_poses > 0 || num_landmarks > 0) {
            ROS_DEBUG("[GraphManager] Optimizer tick - Poses: %zu, Landmarks: %zu",
                      num_poses, num_landmarks);
        }
        
        // Only optimize if we have enough data
        if (num_poses < static_cast<size_t>(min_poses_to_optimize_)) {
            continue;
        }
        
        // --- Phase 2: Build and optimize g2o graph (no lock held) ---
        
        // Clear previous graph
        optimizer_->clear();
        
        // Add pose vertices (VertexSE2)
        for (const auto& kv : poses_copy) {
            auto* vertex = new g2o::VertexSE2();
            vertex->setId(kv.first);
            
            // Set initial estimate
            const g2o::SE2 estimate(kv.second.x, kv.second.y, kv.second.theta);
            vertex->setEstimate(estimate);
            

            // Optimizing poses - only the first one is fixed
            if (first_pose_set_copy && kv.first == first_pose_copy.id) {
                vertex->setFixed(true);
            } else {
                vertex->setFixed(false);
            }
            
            if (!optimizer_->addVertex(vertex)) {
                delete vertex;
                ROS_WARN("[GraphManager] Failed to add pose vertex %d", kv.first);
            }
        }
        
        // Add landmark vertices (VertexPointXY)
        for (const auto& kv : landmarks_copy) {
            auto* vertex = new g2o::VertexPointXY();
            vertex->setId(kv.first);
            
            // Set initial estimate
            const Eigen::Vector2d estimate(kv.second.mean.x(), kv.second.mean.y());
            vertex->setEstimate(estimate);
            
            if (!optimizer_->addVertex(vertex)) {
                delete vertex;
                ROS_WARN("[GraphManager] Failed to add landmark vertex %d", kv.first);
            }
        }
        
        // This time we also add observation edges
        for (const auto& e : odom_edges_copy) {
            auto* edge = new g2o::EdgeSE2();
            edge->setVertex(0, optimizer_->vertex(e.from_id));
            edge->setVertex(1, optimizer_->vertex(e.to_id));
            edge->setMeasurement(g2o::SE2(e.measurement(0), e.measurement(1), e.measurement(2)));
            edge->setInformation(e.information);

            
            auto* rk = new g2o::RobustKernelHuber();
            rk->setDelta(odometry_robust_kernel_delta_);
            edge->setRobustKernel(rk);
            

            optimizer_->addEdge(edge);
        }

        // Add observation edges (EdgeSE2PointXY)
        for (const auto& obs_edge : obs_edges_copy) {
            auto* v0 = optimizer_->vertex(obs_edge.pose_id);
            auto* v1 = optimizer_->vertex(obs_edge.landmark_id);
            
            if (!v0 || !v1) {
                ROS_ERROR("[GraphManager] Observation edge references missing vertex (pose=%d, landmark=%d)",
                          obs_edge.pose_id, obs_edge.landmark_id);
                continue;
            }
            
            auto* edge = new g2o::EdgeSE2PointXY();
            edge->setVertex(0, v0);
            edge->setVertex(1, v1);
            
            // Set measurement (x, y in camera frame)
            edge->setMeasurement(obs_edge.measurement);
            
            // Set information matrix
            edge->setInformation(obs_edge.information);

            // Huber robust kernel prevents outlier observations (e.g. wrong
            // data association) from producing unbounded chi2 contributions.
            auto* rk = new g2o::RobustKernelHuber();
            rk->setDelta(std::sqrt(5.99));  // 95th percentile of chi-squared(2 DOF)
            edge->setRobustKernel(rk);

            if (!optimizer_->addEdge(edge)) {
                delete edge;
                ROS_WARN("[GraphManager] Failed to add observation edge (pose=%d, lm=%d)",
                         obs_edge.pose_id, obs_edge.landmark_id);
            }
        }
        
        // Run optimization
        ROS_INFO("[GraphManager] Running optimization with %zu poses, %zu landmarks, "
                 "%zu odom edges, %zu obs edges",
                 num_poses, num_landmarks, odom_edges_copy.size(), obs_edges_copy.size());
        
        const auto start = std::chrono::steady_clock::now();
        
        optimizer_->initializeOptimization();

        // Log initial chi2 before optimization to diagnose whether inf comes
        // from graph construction (bad info matrices) or solver divergence.
        optimizer_->computeActiveErrors();
        const double init_chi2 = optimizer_->chi2();
        const size_t active_edges = optimizer_->activeEdges().size();
        ROS_INFO("[GraphManager] Pre-optimization: chi2=%.4f, active_edges=%zu", init_chi2, active_edges);

        // Debug: find edges with non-finite chi2 to identify the root cause
        if (!std::isfinite(init_chi2)) {
            int bad_count = 0;
            for (const auto* edge : optimizer_->activeEdges()) {
                if (!std::isfinite(edge->chi2()) && bad_count < 3) {
                    const auto* obs = dynamic_cast<const g2o::EdgeSE2PointXY*>(edge);
                    if (obs) {
                        const auto* vp = static_cast<const g2o::VertexSE2*>(obs->vertex(0));
                        const auto* vl = static_cast<const g2o::VertexPointXY*>(obs->vertex(1));
                        const auto& info = obs->information();
                        ROS_ERROR("[GraphManager] Bad edge chi2=%.4f: pose(%d)=(%.2f,%.2f,%.4f) "
                                  "lm(%d)=(%.2f,%.2f) meas=(%.4f,%.4f) "
                                  "info=[[%.1f,%.1f],[%.1f,%.1f]]",
                                  edge->chi2(),
                                  vp->id(), vp->estimate().translation()[0],
                                  vp->estimate().translation()[1], vp->estimate().rotation().angle(),
                                  vl->id(), vl->estimate()[0], vl->estimate()[1],
                                  obs->measurement()[0], obs->measurement()[1],
                                  info(0,0), info(0,1), info(1,0), info(1,1));
                    }
                    ++bad_count;
                }
            }
        }

        const int iter = optimizer_->optimize(optimizer_iterations_);

        const auto end = std::chrono::steady_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        const double chi2 = optimizer_->chi2();
        const double chi2_per_edge = (active_edges > 0) ? chi2 / static_cast<double>(active_edges) : 0.0;
        ROS_INFO("[GraphManager] Optimization done: iterations=%d, chi2=%.4f (%.4f/edge), active_edges=%zu, time=%ld ms",
                 iter, chi2, chi2_per_edge, active_edges, duration);

        // Guard: skip write-back if optimization diverged or chi2 is unreasonable.
        // Expected chi2/edge for well-associated 2-DOF observations is ~2.0 (chi-squared median).
        // A threshold of 100/edge allows for some outliers without accepting garbage.
        const double kMaxChi2PerEdge = 100.0;
        if (chi2_per_edge > kMaxChi2PerEdge || !std::isfinite(chi2)) {
            ROS_WARN("[GraphManager] Skipping write-back: chi2/edge=%.1f exceeds threshold %.1f",
                     chi2_per_edge, kMaxChi2PerEdge);
            continue;
        }

        // Extract optimized landmark positions
        for (auto& kv : landmarks_copy) {
            auto* vertex = dynamic_cast<g2o::VertexPointXY*>(optimizer_->vertex(kv.first));
            if (vertex) {
                const Eigen::Vector2d& optimized = vertex->estimate();
                kv.second.mean = optimized;
            }
        }

        // Merge optimized landmarks and build remapping for observation edges.
        std::vector<int> to_remove;
        std::unordered_map<int, int> merged_into; // removed_id -> kept_id
        mergeNearbyLandmarks(landmarks_copy, to_remove, merged_into);

        // Extract optimized poses
        for (auto& kv : poses_copy) {
            auto* v = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(kv.first));
            if (v) {
                const g2o::SE2& est = v->estimate();
                kv.second.x = est.translation().x();
                kv.second.y = est.translation().y();
                kv.second.theta = est.rotation().angle();
            }
        }

        // --- Phase 3: Write back optimized values under lock ---
        {
            std::lock_guard<std::mutex> lk(mutex_);

            // Resolve transitive merge chains, e.g. A->B and B->C => A->C.
            auto resolve_merged_id = [&merged_into](int id) {
                int current = id;
                int guard = 0;
                auto it = merged_into.find(current);
                while (it != merged_into.end() && guard < 1000) {
                    current = it->second;
                    it = merged_into.find(current);
                    ++guard;
                }
                return current;
            };

            // Rewire observation edges so they always point to surviving landmarks.
            for (auto& edge : obs_edges_) {
                edge.landmark_id = resolve_merged_id(edge.landmark_id);
            }

            // Remove merged landmarks from main container
            for (int id : to_remove) {
                landmarks_.erase(id);
            }

            for (const auto& kv : landmarks_copy) {
                auto it = landmarks_.find(kv.first);
                if (it != landmarks_.end()) {
                    it->second.mean = kv.second.mean;
                    it->second.cov = kv.second.cov;
                    it->second.observation_count = kv.second.observation_count;
                    it->second.type = kv.second.type;
                }
            }


            // Write optimized poses back to main container
            for (const auto& kv : poses_copy) {
                auto it = poses_.find(kv.first);
                if (it != poses_.end()) {
                    it->second = kv.second;
                }
            }

            // Keep last pose estimate in sync only if the latest pose did not
            // advance during optimization (avoid overwriting newer callback state).
            if (current_pose_id_ == current_pose_id_copy && current_pose_id_copy >= 0) {
                auto it_curr = poses_.find(current_pose_id_copy);
                if (it_curr != poses_.end()) {
                    previous_pose_ = it_curr->second;
                }
            }
            
            // Update map->odom offset from the SAME snapshot pose pair used in
            // optimization. This avoids random jumps when callbacks add newer
            // poses while optimization is running.
            if (current_pose_id_copy >= 0) {
                auto it_opt = poses_.find(current_pose_id_copy);
                if (it_opt != poses_.end()) {
                    const Pose2D& optimized_pose = it_opt->second;
                    const Pose2D& raw_ins_pose = previous_raw_pose_copy;

                    const Pose2D new_offset = calculateOffset(raw_ins_pose, optimized_pose);

                    if (!offset_smoothing_enabled_ || !has_map_to_odom_offset_) {
                        map_to_odom_offset_ = new_offset;
                        has_map_to_odom_offset_ = true;
                    } else {
                        const double alpha = offset_smoothing_alpha_;
                        map_to_odom_offset_.x += alpha * (new_offset.x - map_to_odom_offset_.x);
                        map_to_odom_offset_.y += alpha * (new_offset.y - map_to_odom_offset_.y);

                        const double dtheta = normalizeAngle(new_offset.theta - map_to_odom_offset_.theta);
                        map_to_odom_offset_.theta = normalizeAngle(map_to_odom_offset_.theta + alpha * dtheta);
                    }
                }
            }

            // Mark KD-tree for rebuild after optimization
            kdtree_needs_rebuild_ = true;
        }

        // dupa
        optimized_ = true;
        ROS_INFO("[GraphManager] Graph optimization completed, marking map to be published");
    }

    
    ROS_INFO("[GraphManager] Optimizer loop exiting");
}
