#ifndef RVIZ_MARKERS_PATHPLANNING
#define RVIZ_MARKERS_PATHPLANNING

#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <unordered_map>
#include <memory>

#include "../cone_chain/Cone.h"
#include "../rviz/RvizConesMarkers.h"

class RvizMarkers
{
public:
    RvizMarkers(ros::NodeHandle &nh)
    {
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/path_planning/markers", 10);
        initRvizConesLib();
    };

    void update();

    void addLeftConeChainOrder(const std::vector<Cone> &leftConeChain);
    void addRightConeChainOrder(const std::vector<Cone> &rightConeChain);

    void addBigOrangeCones(const std::unordered_map<Vec2, Cone, Vec2HashFunction> &bigOrangeCones);
    void addSmallOrangeCones(const std::unordered_map<Vec2, Cone, Vec2HashFunction> &orangeCones);
    void addYellowCones(const std::unordered_map<Vec2, Cone, Vec2HashFunction> &cones);
    void addBlueCones(const std::unordered_map<Vec2, Cone, Vec2HashFunction> &cones);
    void addDummyCones(Vec2 leftDummyCone, Vec2 rightDummyCone);
    void addTarget(Vec2 target);
    void addStart(Vec2 start);
    void addPathPoints(const std::vector<Vec2> &path);
    void addRootPathPoints(const std::vector<Vec2> &path);

private:
    ros::Publisher marker_pub;
    visualization_msgs::MarkerArray markersArray;

    visualization_msgs::Marker GetDummyConeTextMarker(Vec2 conePosition, std_msgs::ColorRGBA color, std::string ns, bool isLeft, int idx);
    visualization_msgs::Marker GetTargetTextMarker(Vec2 targetPosition, std_msgs::ColorRGBA color, std::string ns, int idx);

    void addConeChainOrder(const std::vector<Cone> &coneChain, const std::string marker_namespace, std_msgs::ColorRGBA color, int last_cone_chain_size);
};

#endif