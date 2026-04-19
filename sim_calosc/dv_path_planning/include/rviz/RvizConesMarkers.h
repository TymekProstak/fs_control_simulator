#ifndef RVIZ_CONES_MARKERS_HPP
#define RVIZ_CONES_MARKERS_HPP

#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

std::vector<visualization_msgs::Marker> getBigOrangeConeMarkers(float x, float y, int idx);
std::vector<visualization_msgs::Marker> getSmallOrangeConeMarkers(float x, float y, int idx);
std::vector<visualization_msgs::Marker> getYellowConeMarkers(float x, float y, int idx);
std::vector<visualization_msgs::Marker> getBlueConeMarkers(float x, float y, int idx);
std::vector<visualization_msgs::Marker> getPathPointsMarkers(float x, float y, int idx);
std::vector<visualization_msgs::Marker> getRootPathPointsMarkers(float x, float y, int idx);

std::vector<visualization_msgs::Marker> getConeMarkers(float x,
                                                       float y,
                                                       const std::string ns,
                                                       std_msgs::ColorRGBA main_color,
                                                       std_msgs::ColorRGBA strip_color,
                                                       bool isBig,
                                                       int idx);

visualization_msgs::Marker GetConePart(float x,
                                       float y,
                                       float z_offset,
                                       std_msgs::ColorRGBA color,
                                       geometry_msgs::Vector3 scale,
                                       std::string ns,
                                       int shape,
                                       int idx);
visualization_msgs::Marker GetTextMarker(float x, float y, float z, std_msgs::ColorRGBA color, std::string ns, std::string text, int idx);
visualization_msgs::Marker getSquareMarker(float x, float y, float z, std_msgs::ColorRGBA color, std::string ns, int idx);

geometry_msgs::Vector3 operator*(const geometry_msgs::Vector3 &vec3, float scale);
void initRvizConesLib();

#endif