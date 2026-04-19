#pragma once

#include <Eigen/Dense>
#include <string>
#include <atomic>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Landmark {
    int id = -1;
    Eigen::Vector2d mean = {0,0};
    Eigen::Matrix2d cov = Eigen::Matrix2d::Identity();
    int observation_count = 0;
    bool is_permanent = false;
    std::string type = "unknown";
};

struct Pose2D {
    int id = -1;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
};

class IdGenerator {
public:
    IdGenerator(int start_even = 0, int start_odd = 1)
        : next_pose_(start_even), next_landmark_(start_odd)
    {}

    int nextPoseId() { return next_pose_.fetch_add(2); }
    int nextLandmarkId() { return next_landmark_.fetch_add(2); }
private:
    std::atomic<int> next_pose_;
    std::atomic<int> next_landmark_;
};

/// Normalize angle to [-pi, pi] safely (no loop, handles NaN)
inline double normalizeAngle(const double angle)
{
    return std::remainder(angle, 2.0 * M_PI);
}

/// Extract yaw from a quaternion (x, y, z, w)
inline double quaternionToYaw(const double qx, const double qy, const double qz, const double qw)
{
    // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

/**
 * @brief Aplikuje poprawkę (offset) SE2 do surowej pozy.
 * Wykorzystywane do zamiany współrzędnych Odom -> Map.
 */
inline Pose2D applyOffset(const Pose2D& raw, const Pose2D& offset)
{
    Pose2D corrected;
    corrected.id = raw.id;
    
    const double cos_off = std::cos(offset.theta);
    const double sin_off = std::sin(offset.theta);
    
    // Rotacja wektora pozycji o kąt offsetu i dodanie przesunięcia
    corrected.x = (raw.x * cos_off - raw.y * sin_off) + offset.x;
    corrected.y = (raw.x * sin_off + raw.y * cos_off) + offset.y;
    
    // Korekta orientacji
    corrected.theta = normalizeAngle(raw.theta + offset.theta);
    
    return corrected;
}

/**
 * @brief Oblicza offset SE2 taki, że: target = applyOffset(source, offset).
 * Wykorzystywane w pętli optymalizatora do wyznaczenia poprawki Map-to-Odom.
 */
inline Pose2D calculateOffset(const Pose2D& source, const Pose2D& target)
{
    Pose2D offset;
    offset.theta = normalizeAngle(target.theta - source.theta);
    
    const double cos_off = std::cos(offset.theta);
    const double sin_off = std::sin(offset.theta);
    
    // Wyznaczenie przesunięcia x i y po uwzględnieniu różnicy kątów
    offset.x = target.x - (source.x * cos_off - source.y * sin_off);
    offset.y = target.y - (source.x * sin_off + source.y * cos_off);
    
    return offset;
}