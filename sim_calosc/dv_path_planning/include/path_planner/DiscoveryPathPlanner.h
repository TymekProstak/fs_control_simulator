#ifndef AUTOCROSS_PATH_PLANNER_H
#define AUTOCROSS_PATH_PLANNER_H

#include "PathPlanner.h"
#include "MissionState.h"

class DiscoveryPathPlanner : public PathPlanner
{
public:
    DiscoveryPathPlanner();

    void plan(pcl::PointCloud<pcl::PointXYZL> &cones, BolideDescriptor &bolide) override;
    void plan(const dv_interfaces::Cones &cones_msg, BolideDescriptor &bolide) override;

private:
    void publishRviz() override;
    void updateState();
    
    void plan_general();

    MissionState state = MissionState::LOOKING_FOR_START;
};

#endif // AUTOCROSS_PATH_PLANNER_H