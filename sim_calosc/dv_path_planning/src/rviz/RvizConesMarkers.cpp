#include "../../include/rviz/RvizConesMarkers.h"

std_msgs::ColorRGBA yellow_color;
std_msgs::ColorRGBA blue_color;
std_msgs::ColorRGBA light_blue_color;
std_msgs::ColorRGBA orange_color;
std_msgs::ColorRGBA black_color;
std_msgs::ColorRGBA white_color;
std_msgs::ColorRGBA green_color;

geometry_msgs::Vector3 scale_small_cone_base;
geometry_msgs::Vector3 scale_small_cone_lower;
geometry_msgs::Vector3 scale_small_cone_middle;
geometry_msgs::Vector3 scale_small_cone_upper;

float z_offset_base;
float z_offset_lower;
float z_offset_middle;
float z_offset_upper;

geometry_msgs::Vector3 scale_big_cone_base;
geometry_msgs::Vector3 scale_big_cone_lower;
geometry_msgs::Vector3 scale_big_cone_middle;
geometry_msgs::Vector3 scale_big_cone_upper;

const int CUBE_SHAPE = visualization_msgs::Marker::CUBE;
const int CYLINDER_SHAPE = visualization_msgs::Marker::CYLINDER;

void initRvizConesLib()
{
    yellow_color.r = 1;
    yellow_color.g = 1;
    yellow_color.b = 0;
    yellow_color.a = 1;

    blue_color.r = 0.4;
    blue_color.g = 0.7;
    blue_color.b = 1;
    blue_color.a = 1;

    green_color.g = 1;
    green_color.a = 1;

    orange_color.r = 1;
    orange_color.g = 0.5;
    orange_color.b = 0;
    orange_color.a = 1;

    white_color.r = 1;
    white_color.g = 1;
    white_color.b = 1;
    white_color.a = 1;

    black_color.r = 0;
    black_color.g = 0;
    black_color.b = 0;
    black_color.a = 1;

    scale_small_cone_base.x = 0.225;
    scale_small_cone_base.y = 0.225;
    scale_small_cone_base.z = 0.025;

    scale_small_cone_lower.x = 0.155;
    scale_small_cone_lower.y = 0.155;
    scale_small_cone_lower.z = 0.1;

    scale_small_cone_middle.x = 0.115;
    scale_small_cone_middle.y = 0.115;
    scale_small_cone_middle.z = 0.1;

    scale_small_cone_upper.x = 0.0725;
    scale_small_cone_upper.y = 0.0725;
    scale_small_cone_upper.z = 0.1;

    z_offset_base = 0.0;
    z_offset_lower = scale_small_cone_base.z;
    z_offset_middle = z_offset_lower + scale_small_cone_lower.z;
    z_offset_upper = z_offset_middle + scale_small_cone_middle.z;

    scale_big_cone_base.x = 1.5 * scale_small_cone_base.x;
    scale_big_cone_base.y = 1.5 * scale_small_cone_base.y;
    scale_big_cone_base.z = 1.5 * scale_small_cone_base.z;

    scale_big_cone_lower.x = 1.5 * scale_small_cone_lower.x;
    scale_big_cone_lower.y = 1.5 * scale_small_cone_lower.y;
    scale_big_cone_lower.z = 1.5 * scale_small_cone_lower.z;

    scale_big_cone_middle.x = 1.5 * scale_small_cone_middle.x;
    scale_big_cone_middle.y = 1.5 * scale_small_cone_middle.y;
    scale_big_cone_middle.z = 1.5 * scale_small_cone_middle.z;

    scale_big_cone_upper.x = 1.5 * scale_small_cone_upper.x;
    scale_big_cone_upper.y = 1.5 * scale_small_cone_upper.y;
    scale_big_cone_upper.z = 1.5 * scale_small_cone_upper.z;
}

std::vector<visualization_msgs::Marker> getBigOrangeConeMarkers(float x, float y, int idx)
{
    return getConeMarkers(x, y, "BigOrangeCone", orange_color, white_color, true, idx);
}

std::vector<visualization_msgs::Marker> getSmallOrangeConeMarkers(float x, float y, int idx)
{
    return getConeMarkers(x, y, "SmallOrangeCone", orange_color, white_color, false, idx);
}

std::vector<visualization_msgs::Marker> getYellowConeMarkers(float x, float y, int idx)
{
    return getConeMarkers(x, y, "YellowCone", yellow_color, black_color, true, idx);
}

std::vector<visualization_msgs::Marker> getBlueConeMarkers(float x, float y, int idx)
{
    return getConeMarkers(x, y, "BlueCone", blue_color, white_color, true, idx);
}

std::vector<visualization_msgs::Marker> getPathPointsMarkers(float x, float y, int idx){
    std::vector<visualization_msgs::Marker> markers;

    visualization_msgs::Marker txtMarker = GetTextMarker(x, y, 0.6, green_color, "PathPoints_text", std::to_string(idx), idx);
    markers.push_back(txtMarker);

    visualization_msgs::Marker squareMarker = getSquareMarker(x, y, 0, green_color, "PathPoints", idx);
    markers.push_back(squareMarker);
    return markers;
}

std::vector<visualization_msgs::Marker> getRootPathPointsMarkers(float x, float y, int idx){
    std::vector<visualization_msgs::Marker> markers;

    visualization_msgs::Marker txtMarker = GetTextMarker(x, y, 0.6, green_color, "RootPathPoints_text", std::to_string(idx), idx);
    markers.push_back(txtMarker);

    std_msgs::ColorRGBA color;
    color.r = 0.5;
    color.g = 0.5;
    color.b = 0.5;
    color.a = 1.0;
    visualization_msgs::Marker squareMarker = getSquareMarker(x, y, 0, color, "RootPathPoints", idx);
    squareMarker.scale.x = squareMarker.scale.y = 1;
    markers.push_back(squareMarker);
    return markers;
}

visualization_msgs::Marker getSquareMarker(float x, float y, float z, std_msgs::ColorRGBA color, std::string ns, int idx){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();

    marker.ns = ns;
    marker.id = idx;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    const float width = 0.2;
    marker.scale.x = width;
    marker.scale.y = width;

    marker.color = color;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    marker.points.push_back(point);

    return marker;
}

visualization_msgs::Marker GetTextMarker(float x, float y, float z, std_msgs::ColorRGBA color, std::string ns, std::string text, int idx)
{
    std::stringstream ss;
    visualization_msgs::Marker marker_text;
    marker_text.header.frame_id = "map";
    marker_text.header.stamp = ros::Time::now();
    marker_text.ns = ns;
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::Marker::ADD;

    marker_text.id = idx;
    marker_text.text = text;
    marker_text.scale.z = 0.2;
    marker_text.color = color;
    marker_text.pose.orientation.x = 0;
    marker_text.pose.orientation.y = 0;
    marker_text.pose.orientation.z = 0;
    marker_text.pose.orientation.w = 1;
    marker_text.pose.position.x = x;
    marker_text.pose.position.y = y;
    marker_text.pose.position.z = z;

    return marker_text;
}

std::vector<visualization_msgs::Marker> getConeMarkers(float x, float y, const std::string ns, std_msgs::ColorRGBA main_color, std_msgs::ColorRGBA strip_color, bool isBig, int idx)
{
    std::vector<visualization_msgs::Marker> markers;

    const auto scale = isBig ? 1.5 : 1.0;

    visualization_msgs::Marker cone_base = GetConePart(x, y, z_offset_base, main_color, scale_small_cone_base * scale, ns, CUBE_SHAPE, 4 * idx);
    visualization_msgs::Marker cone_lower = GetConePart(x, y, z_offset_lower, main_color, scale_small_cone_lower * scale, ns, CYLINDER_SHAPE, 4 * idx + 1);
    visualization_msgs::Marker cone_middle = GetConePart(x, y, z_offset_middle, strip_color, scale_small_cone_middle * scale, ns, CYLINDER_SHAPE, 4 * idx + 2);
    visualization_msgs::Marker cone_upper = GetConePart(x, y, z_offset_upper, main_color, scale_small_cone_upper * scale, ns, CYLINDER_SHAPE, 4 * idx + 3);

    markers.push_back(cone_base);
    markers.push_back(cone_lower);
    markers.push_back(cone_middle);
    markers.push_back(cone_upper);

    return markers;
}

visualization_msgs::Marker GetConePart(float x,
                                       float y,
                                       float z_offset,
                                       std_msgs::ColorRGBA color,
                                       geometry_msgs::Vector3 scale,
                                       std::string ns,
                                       int shape,
                                       int idx)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale = scale;
    marker.color = color;
    marker.id = idx;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.5 * marker.scale.z + z_offset;

    return marker;
}

geometry_msgs::Vector3 operator*(const geometry_msgs::Vector3 &vec3, float scale)
{
    geometry_msgs::Vector3 out_vec3;
    out_vec3.x = vec3.x * scale;
    out_vec3.y = vec3.y * scale;
    out_vec3.z = vec3.z * scale;
    return out_vec3;
}
