#ifndef TRACKDRIVE_PATH_PLANNER_H
#define TRACKDRIVE_PATH_PLANNER_H

#include "PathPlanner.h"
#include "MissionState.h"

class TrackdrivePathPlanner : public PathPlanner
{
public:
    TrackdrivePathPlanner();

    void plan(pcl::PointCloud<pcl::PointXYZL> &cones, BolideDescriptor &bolide) override;
    void plan(const dv_interfaces::Cones &cones_msg, BolideDescriptor &bolide) override;

private:
    void publishRviz() override;
    void updateState();

    void plan_general();
    void post_PoseCallbackCallback(const ros::TimerEvent &event) override;

    void initialize_lap_counting(); // HAS TO BE CALLED BEFORE INITIALIZING cl

    ros::Timer lap_timer;
    Vec2 finish_center = Vec2(0, 0);
    // Average position for finish cones on each side
    Vec2 finish_right = Vec2(0, 0);
    Vec2 finish_left = Vec2(0, 0);
    int lap_count = 0;

    MissionState state = MissionState::LOOKING_FOR_START;
};

#endif // TRACKDRIVE_PATH_PLANNER_H