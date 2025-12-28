#include <ros/ros.h>
#include <dv_interfaces/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct XY {
  double x{};
  double y{};
};

static bool parseTrackCsv(const std::string& file_path, std::vector<XY>& out_points, std::string& err)
{
  out_points.clear();

  std::ifstream in(file_path);
  if (!in.is_open()) {
    err = "cannot open file: " + file_path;
    return false;
  }

  std::string line;
  bool any_data = false;
  while (std::getline(in, line)) {
    // skip empty lines
    if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;

    // Accept: "x,y" or ";"-separated or with extra columns.
    // Also tolerate an optional header by skipping first non-numeric line.
    std::string normalized = line;
    for (char& c : normalized) {
      if (c == ';') c = ',';
    }

    std::stringstream ss(normalized);
    std::string token_x, token_y;
    if (!std::getline(ss, token_x, ',')) continue;
    if (!std::getline(ss, token_y, ',')) continue;

    try {
      size_t idx_x = 0, idx_y = 0;
      double x = std::stod(token_x, &idx_x);
      double y = std::stod(token_y, &idx_y);
      (void)idx_x;
      (void)idx_y;

      out_points.push_back({x, y});
      any_data = true;
    } catch (const std::exception&) {
      // likely header or malformed line; skip
      continue;
    }
  }

  if (!any_data || out_points.size() < 2) {
    err = "track csv has too few valid points (<2): " + file_path;
    return false;
  }

  return true;
}

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "false_dv_path_planning_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Match dv_control wrapper.cpp subscription:
  // path_sub = nh.subscribe("/path_planning/path", ...)
  ros::Publisher path_pub = nh.advertise<dv_interfaces::Path>("/path_planning/path", 1, true /* latch */);

  std::string track_file;
  // In sim.launch params are set under the node namespace with <param name="track_file" ...>
  // So both private (~track_file) and global (/track_file) are tried (private first).
  if (!pnh.getParam("track_file", track_file)) {
    (void)nh.getParam("track_file", track_file);
  }

  if (track_file.empty()) {
    ROS_ERROR_STREAM("[false_dv_path_planning_node] Missing param 'track_file' (expected CSV with x,y points).");
    return 1;
  }

  double publish_rate_hz = 10.0;
  pnh.param("publish_rate", publish_rate_hz, 10.0);

  std::string frame_id = "map";
  pnh.param<std::string>("frame_id", frame_id, "map");

  std::vector<XY> pts;
  std::string err;
  if (!parseTrackCsv(track_file, pts, err)) {
    ROS_ERROR_STREAM("[false_dv_path_planning_node] Failed to load track: " << err);
    return 1;
  }

  dv_interfaces::Path msg;
  msg.full_path_enabled = true; // as requested: full/global path flag

  msg.path.header.frame_id = frame_id;
  msg.path.header.stamp = ros::Time::now();
  msg.path.poses.reserve(pts.size());

  for (const auto& p : pts) {
    geometry_msgs::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0; // no yaw provided
    msg.path.poses.push_back(pose);
  }

  ROS_INFO_STREAM("[false_dv_path_planning_node] Loaded " << pts.size() << " points from: " << track_file);
  ROS_INFO_STREAM("[false_dv_path_planning_node] Publishing latched dv_interfaces/Path on /path_planning/path (full_path_enabled=true)");

  ros::Rate rate(std::max(0.1, publish_rate_hz));
  while (ros::ok()) {
    msg.path.header.stamp = ros::Time::now();
    path_pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
