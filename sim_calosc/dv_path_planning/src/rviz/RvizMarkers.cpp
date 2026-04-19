#include "../include/rviz/RvizMarkers.h"

void RvizMarkers::addBigOrangeCones(const std::unordered_map<Vec2, Cone, Vec2HashFunction> &bigOrangeCones)
{
    static int lastMaxIdx = 0;

    // overriding existing markers with given ids (or creating brand new one) with up to date markers
    int conesCounter = 0;
    for (const auto &bigOrangeCone : bigOrangeCones)
    {
        std::vector<visualization_msgs::Marker> markers = getBigOrangeConeMarkers(bigOrangeCone.first.x, bigOrangeCone.first.y, conesCounter);
        for (visualization_msgs::Marker &m : markers)
            markersArray.markers.push_back(m);
        conesCounter++;
    }

    // Remove previous markers that are were not overwritten
    if (conesCounter < lastMaxIdx)
    {
        for (int idx = conesCounter; idx < lastMaxIdx; ++idx)
        {
            visualization_msgs::Marker marker;
            marker.ns = "BigOrangeCone";
            marker.id = idx;
            marker.action = visualization_msgs::Marker::DELETE;
            markersArray.markers.push_back(marker);
        }
    }

    lastMaxIdx = conesCounter;
}

void RvizMarkers::addSmallOrangeCones(const std::unordered_map<Vec2, Cone, Vec2HashFunction> &smallOrangeCones)
{
    static int lastMaxIdx = 0;

    // overriding existing markers with given ids (or creating brand new one) with up to date markers
    int conesCounter = 0;
    for (const auto &smallOrangeCone : smallOrangeCones)
    {
        std::vector<visualization_msgs::Marker> markers = getBigOrangeConeMarkers(smallOrangeCone.first.x, smallOrangeCone.first.y, conesCounter);
        for (visualization_msgs::Marker &m : markers)
            markersArray.markers.push_back(m);
        conesCounter++;
    }

    // Remove previous markers that are were not overwritten
    // It only happens if the number of markers is bigger that the current number of small orange cones
    if (conesCounter < lastMaxIdx)
    {
        for (int idx = conesCounter + 1; idx <= lastMaxIdx; idx++)
        {
            visualization_msgs::Marker marker;
            marker.ns = "SmallOrangeCone";
            marker.id = idx;
            marker.action = visualization_msgs::Marker::DELETE;
            markersArray.markers.push_back(marker);
        }
    }

    lastMaxIdx = conesCounter;
}

void RvizMarkers::addYellowCones(const std::unordered_map<Vec2, Cone, Vec2HashFunction> &yellowCones)
{
    static int lastMaxIdx = 0;

    // overriding existing markers with given ids (or creating brand new one) with up to date markers
    int conesCounter = 0;
    for (const auto &yellowCone : yellowCones)
    {
        std::vector<visualization_msgs::Marker> markers = getYellowConeMarkers(yellowCone.first.x, yellowCone.first.y, conesCounter);
        for (visualization_msgs::Marker &m : markers)
            markersArray.markers.push_back(m);
        conesCounter++;
    }

    // Remove previous markers that are were not overwritten
    if (conesCounter < lastMaxIdx)
    {
        for (int idx = conesCounter; idx < lastMaxIdx; ++idx)
        {
            visualization_msgs::Marker marker;
            marker.ns = "YellowCone";
            marker.id = idx;
            marker.action = visualization_msgs::Marker::DELETE;
            markersArray.markers.push_back(marker);
        }
    }

    lastMaxIdx = conesCounter;
}

void RvizMarkers::addBlueCones(const std::unordered_map<Vec2, Cone, Vec2HashFunction> &blueCones)
{
    static int lastMaxIdx = 0;

    // overriding existing markers with given ids (or creating brand new one) with up to date markers
    int conesCounter = 0;
    for (const auto &blueCone : blueCones)
    {
        std::vector<visualization_msgs::Marker> markers = getBlueConeMarkers(blueCone.first.x, blueCone.first.y, conesCounter);
        for (visualization_msgs::Marker &m : markers)
            markersArray.markers.push_back(m);
        conesCounter++;
    }

    // Remove previous markers that are were not overwritten
    if (conesCounter < lastMaxIdx)
    {
        for (int idx = conesCounter; idx < lastMaxIdx; ++idx)
        {
            visualization_msgs::Marker marker;
            marker.ns = "BlueCone";
            marker.id = idx;
            marker.action = visualization_msgs::Marker::DELETE;
            markersArray.markers.push_back(marker);
        }
    }

    lastMaxIdx = conesCounter;
}

void RvizMarkers::addDummyCones(Vec2 leftSourceCone, Vec2 rightSourceCone)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();

    const std::string ns = "_dummy";
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    const float width = 0.2;
    marker.scale.x = width;
    marker.scale.y = width;

    std_msgs::ColorRGBA color;
    color.b = 1;
    color.r = 0.7;
    color.a = 0.7;
    marker.color = color;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    geometry_msgs::Point leftSourcePoint;
    geometry_msgs::Point rightSourcePoint;

    leftSourcePoint.x = leftSourceCone.x;
    leftSourcePoint.y = leftSourceCone.y;
    leftSourcePoint.z = 0.0;
    marker.points.push_back(leftSourcePoint);

    rightSourcePoint.x = rightSourceCone.x;
    rightSourcePoint.y = rightSourceCone.y;
    rightSourcePoint.z = 0.0;
    marker.points.push_back(rightSourcePoint);

    markersArray.markers.push_back(marker);

    // marker labels
    visualization_msgs::Marker marker_text_right = GetDummyConeTextMarker(rightSourceCone, color, ns, false, 0);
    markersArray.markers.push_back(marker_text_right);

    visualization_msgs::Marker marker_text_left = GetDummyConeTextMarker(leftSourceCone, color, ns, true, 1);
    markersArray.markers.push_back(marker_text_left);
}

visualization_msgs::Marker RvizMarkers::GetDummyConeTextMarker(Vec2 conePosition, std_msgs::ColorRGBA color, std::string ns, bool isLeft, int idx)
{
    std::stringstream ss;
    visualization_msgs::Marker marker_text;
    marker_text.header.frame_id = "map";
    marker_text.header.stamp = ros::Time::now();
    marker_text.ns = ns + std::string("_dummy_text");
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::Marker::ADD;

    marker_text.id = idx;
    marker_text.text = isLeft ? "LEFT dummy" : "RIGHT dummy";
    marker_text.scale.z = 0.2;
    marker_text.color = color;
    marker_text.pose.orientation.x = 0;
    marker_text.pose.orientation.y = 0;
    marker_text.pose.orientation.z = 0;
    marker_text.pose.orientation.w = 1;
    marker_text.pose.position.x = conePosition.x;
    marker_text.pose.position.y = conePosition.y;
    marker_text.pose.position.z = 0.7;

    return marker_text;
}

void RvizMarkers::addTarget(Vec2 target)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();

    const std::string ns = "_target";
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    const float width = 0.2;
    marker.scale.x = width;
    marker.scale.y = width;

    std_msgs::ColorRGBA color;
    color.b = 1;
    color.r = 1;
    color.a = 0.7;
    marker.color = color;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    geometry_msgs::Point targetPoint;

    targetPoint.x = target.x;
    targetPoint.y = target.y;
    targetPoint.z = 0.0;
    marker.points.push_back(targetPoint);

    markersArray.markers.push_back(marker);

    // marker labels
    visualization_msgs::Marker marker_text_left = GetTargetTextMarker(target, color, ns, 1);
    markersArray.markers.push_back(marker_text_left);
}

void RvizMarkers::addStart(Vec2 start)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();

    const std::string ns = "_start";
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    const float width = 0.4;
    marker.scale.x = width;
    marker.scale.y = width;

    std_msgs::ColorRGBA color;
    color.g = 1;
    color.r = 1;
    color.a = 0.7;
    marker.color = color;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    geometry_msgs::Point startPoint;
    startPoint.x = start.x;
    startPoint.y = start.y;
    marker.points.push_back(startPoint);

    markersArray.markers.push_back(marker);

    // marker labels
    // visualization_msgs::Marker marker_text_left = GetTargetTextMarker(start, color, ns, 1);
    // markersArray.markers.push_back(marker_text_left);
}

visualization_msgs::Marker RvizMarkers::GetTargetTextMarker(Vec2 targetPosition, std_msgs::ColorRGBA color, std::string ns, int idx)
{
    std::stringstream ss;
    visualization_msgs::Marker marker_text;
    marker_text.header.frame_id = "map";
    marker_text.header.stamp = ros::Time::now();
    marker_text.ns = ns + std::string("_target_text");
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::Marker::ADD;

    marker_text.id = idx;
    marker_text.text = "target";
    marker_text.scale.z = 0.2;
    marker_text.color = color;
    marker_text.pose.orientation.x = 0;
    marker_text.pose.orientation.y = 0;
    marker_text.pose.orientation.z = 0;
    marker_text.pose.orientation.w = 1;
    marker_text.pose.position.x = targetPosition.x;
    marker_text.pose.position.y = targetPosition.y;
    marker_text.pose.position.z = 0.7;

    return marker_text;
}

void RvizMarkers::addLeftConeChainOrder(const std::vector<Cone> &leftConeChain)
{
    std_msgs::ColorRGBA ligth_blue;
    ligth_blue.r = 0.6;
    ligth_blue.g = 0.8;
    ligth_blue.b = 1;
    ligth_blue.a = 1;

    static int last_cone_chain_size = 0;
    addConeChainOrder(leftConeChain, "LeftConeChain", ligth_blue, last_cone_chain_size);
    last_cone_chain_size = leftConeChain.size();
}

void RvizMarkers::addRightConeChainOrder(const std::vector<Cone> &rightConeChain)
{
    std_msgs::ColorRGBA yellow;
    yellow.r = 1;
    yellow.g = 1;
    yellow.a = 1;

    static int last_cone_chain_size = 0;
    addConeChainOrder(rightConeChain, "RightConeChain", yellow, last_cone_chain_size);
    last_cone_chain_size = rightConeChain.size();
}

void RvizMarkers::addConeChainOrder(const std::vector<Cone> &coneChain, const std::string marker_namespace, std_msgs::ColorRGBA color, int last_cone_chain_size)
{
    // Remove previous markers
    for (int idx = 0; idx < last_cone_chain_size; idx++)
    {
        visualization_msgs::Marker marker;
        marker.ns = marker_namespace;
        marker.id = idx;
        marker.action = visualization_msgs::Marker::DELETE;
        markersArray.markers.push_back(marker);
    }

    // Add new markers
    for (int idx = 0; idx < coneChain.size(); ++idx)
    {
        visualization_msgs::Marker marker = GetTextMarker(coneChain[idx].getPosition().x, coneChain[idx].getPosition().y, 0.6, color, marker_namespace, std::to_string(idx), idx);
        markersArray.markers.push_back(marker);
    }
}

void RvizMarkers::addPathPoints(const std::vector<Vec2> &path)
{
    // Remove previous markers
    static int last_max_idx = 0;
    for (int idx = 0; idx < last_max_idx; idx++)
    {
        visualization_msgs::Marker marker, marker_txt;
        marker.ns = "PathPoints";
        marker.id = idx;
        marker.action = visualization_msgs::Marker::DELETE;
        markersArray.markers.push_back(marker);

        marker_txt.ns = "PathPoints_text";
        marker_txt.id = idx;
        marker_txt.action = visualization_msgs::Marker::DELETE;
        markersArray.markers.push_back(marker_txt);
    }

    // Add new markers
    for (int idx = 0; idx < path.size(); ++idx)
    {
        std::vector<visualization_msgs::Marker> markers = getPathPointsMarkers(path[idx].x, path[idx].y, idx);
        for (visualization_msgs::Marker &m : markers)
            markersArray.markers.push_back(m);
    }

    last_max_idx = path.size();
}

void RvizMarkers::addRootPathPoints(const std::vector<Vec2> &path)
{
    // Remove previous markers
    static int last_max_idx = 0;
    for (int idx = 0; idx < last_max_idx; idx++)
    {
        visualization_msgs::Marker marker, marker_txt;
        marker.ns = "RootPathPoints";
        marker.id = idx;
        marker.action = visualization_msgs::Marker::DELETE;
        markersArray.markers.push_back(marker);

        marker_txt.ns = "RootPathPoints_text";
        marker_txt.id = idx;
        marker_txt.action = visualization_msgs::Marker::DELETE;
        markersArray.markers.push_back(marker_txt);
    }

    for (int idx = 0; idx < last_max_idx; idx++)
    {
        visualization_msgs::Marker marker, marker_txt;
        marker.ns = "RootPathPoints";
        marker.id = idx;
        marker.action = visualization_msgs::Marker::DELETE;
        markersArray.markers.push_back(marker);
    }

    // Add new markers
    for (int idx = 0; idx < path.size(); ++idx)
    {
        std::vector<visualization_msgs::Marker> markers = getRootPathPointsMarkers(path[idx].x, path[idx].y, idx);
        for (visualization_msgs::Marker &m : markers)
            markersArray.markers.push_back(m);
    }

    last_max_idx = path.size();
}

void RvizMarkers::update()
{
    marker_pub.publish(markersArray);
    markersArray = visualization_msgs::MarkerArray();
}
