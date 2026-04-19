#include <memory>
#include "../../include/path_planner/DiscoveryPathPlanner.h"
#include <common/config.h>
#include "cone_chain/ConeLookup.h"
#include "../../include/path_planner/PathPlanner.h"
#if defined(PERFORM_PARAMETER_OPTIMIZATION) && __has_include(<ensmallen.hpp>)
#include <ensmallen.hpp>
#endif

DiscoveryPathPlanner::DiscoveryPathPlanner() : PathPlanner()
{

}

// Sub-step method
void DiscoveryPathPlanner::plan_general() {

    ROS_INFO("Loaded cones\n");

    if (cl.GetLeftCones().empty() || cl.GetRightCones().empty()) {
        ROS_WARN("Bad Cones!\n");
        return;
    }

    closed_loop = false;

    // save starting position for later (to check if is looping)
    Vec2 start_pos = bolide.getPosition();
    // Find cones closes to the car and start from them
    const Cone* leftCone = cl.LookupClosestLeftCone(start_pos);
    const Cone* rightCone = cl.LookupClosestRightCone(start_pos);
    const Cone* last_leftCone = nullptr, *last_rightCone = nullptr;

    // const Vec2 across_track_vec = leftCones[leftCone].getPosition() - rightCones[rightCone].getPosition();
    // const Vec2 along_track_vec = Vec2(across_track_vec.y, -across_track_vec.x) * TRACK_STEP_SIZE;
    // Vec2 next_pos = Vec2(leftCones[leftCone].getPosition() + along_track_vec);
    // while (leftCones[leftCone].getType() == ORANGE_BIG || leftCones[leftCone].getType() == ORANGE_SMALL) {
    //     leftCone = cl.LookupClosestLeftCone(next_pos) - leftCones.data();
    //     next_pos += along_track_vec;
    // }

    //rightCone = cl.LookupClosestRightCone(leftCones[leftCone].getPosition()) - rightCones.data();

    Vec2 trackPoint = (leftCone->getPosition() + rightCone->getPosition()) / 2;
    const Vec2 start_track_point = trackPoint;

    //path.push_back(trackPoint); // Add first point
    // Following a point in the center of the track find 2 closest cones (left and right),
    // add middle point to path, move the center point forward. Repeat
    int iter = 0;
    // while ( leftCone < leftCones.size() && leftCones[leftCone].getType() != ORANGE_BIG &&
    //         rightCone < rightCones.size() && rightCones[rightCone].getType() != ORANGE_BIG &&
    //         iter < 1000) {
    bool left_start = false;
    while (iter < PATH_PLANNING_MAX_ITERATIONS) {

        //ROS_INFO("Path-Planning iteration %i\n", iter);

        trackPoint = (leftCone->getPosition() + rightCone->getPosition()) / 2;
        if (!path.empty() && (trackPoint - path.back()).length_squared() > (MAX_DISTANCE_BETWEEN_PATH_POINTS * MAX_DISTANCE_BETWEEN_PATH_POINTS)) {
            break;
        }

        // Break early if looping
        if ((start_track_point - trackPoint).length_squared() < (POSITION_PROXIMITY_THRESHOLD * POSITION_PROXIMITY_THRESHOLD)) { // Distance check
            if (left_start) { // Alignment check (is the point behind)
                if ((path[0] - path.back()).length_squared() < 0.001) {
                    path.pop_back(); // Pop the last point if the first point the same
                }
                ROS_INFO("Looping, Exiting");
                closed_loop = true;
                break;
            }
        }
        else {
            left_start = true;
        }

        path.push_back(trackPoint);

        //trackPoint = (leftCones[leftCone].getPosition() + rightCones[rightCone].getPosition()) / 2;

        // Move track point by a given amount
        // const Vec2 track_step = (leftCones[leftCone + 1].getPosition() - leftCones[leftCone].getPosition() +
        //                     (rightCones[rightCone + 1].getPosition() - rightCones[rightCone].getPosition())) / 2;
        const Vec2 across_track_vector = leftCone->getPosition() - rightCone->getPosition();
        //const Vec2 track_step = (iter == 0 ? Vec2(across_track_vector.y, -across_track_vector.x) : (path.back() - path[path.size()-2])) * TRACK_STEP_SIZE;
        Vec2 track_step = Vec2(across_track_vector.y, -across_track_vector.x) * TRACK_STEP_SIZE; // Vector along the track
        // IDEA: Use LERP between this and the next track vector
        trackPoint += track_step;

        if (path.size() >= 2 && (track_step.x * track_step.x + track_step.y * track_step.y) < 0.00001) {
            // Step vector too short
            track_step = path.back() - path[path.size()-2];
        }

        // Find new closest cones
        const Cone* new_leftCone = cl.LookupClosestLeftCone(trackPoint);
        const Cone* new_rightCone = cl.LookupClosestRightCone(trackPoint);
        int iter_1 = 0; // Safety check
        while (new_leftCone == leftCone && new_rightCone == rightCone && iter_1 < 20) {
            trackPoint += track_step;
            new_leftCone = cl.LookupClosestLeftCone(trackPoint);
            new_rightCone = cl.LookupClosestRightCone(trackPoint);
            iter_1++;
        }

        // Dont skip 2 cones at once to stay consistent
        if (leftCone != new_leftCone && rightCone != new_rightCone) {
            new_leftCone = leftCone;
        }

        // Could not find the next cone, or is looping
        if (iter_1 >= 20 || new_leftCone == last_leftCone || new_rightCone == last_rightCone) {
            break;
        }

        // Assign only if changed
        last_leftCone = leftCone != new_leftCone ? leftCone : last_leftCone;
        last_rightCone = rightCone != new_rightCone ? rightCone : last_rightCone;

        leftCone = new_leftCone;
        rightCone = new_rightCone;

        iter++;
    }
}

void DiscoveryPathPlanner::plan(pcl::PointCloud<pcl::PointXYZL> &cones, BolideDescriptor &bolide)
{
    path.clear();

    if (cones.size() <= 1) {
        return;
    }

    cl = ConeLookup(cones);

    plan_general();

    smoothPath(path, 0.9081, 0.2356, 0.0230, 0.6117, 0);
    //smoothLowpassPreserveCorners(path, 0.4, 3, 0.2f, 1.0f);

    if (INTERPOLATION_TYPE != 0 && path.size() > 2) {
        root_path = path;
        switch (INTERPOLATION_TYPE) {
            case 1:
                interpolateLinear(&path, NUMBER_OF_PATH_POINTS);
                break;
            case 2:
                interpolateAkimaSpline(path, GN_DISTANCE_BETWEEN_POINTS, NUMBER_OF_PATH_POINTS);
                break;
            case 3:
                interpolateCubicSpline(path, GN_DISTANCE_BETWEEN_POINTS, NUMBER_OF_PATH_POINTS);
                break;
            case 4:
                interpolateGaussNewton(path, GN_DISTANCE_BETWEEN_POINTS, NUMBER_OF_PATH_POINTS, GN_POLYNOMIAL_DEGREE, GN_MAX_ITER, GN_PRECISION);
                break;
            default:
                break;
        }
    }
    ROS_INFO("Path size: %lu\n", path.size());
    updateState();
}

void DiscoveryPathPlanner::plan(const dv_interfaces::Cones &cones_msg, BolideDescriptor &bolide)
{
    path.clear();

    if (cones_msg.cones.size() <= 1) {
        return;
    }

    cl = ConeLookup(cones_msg);

    // ~50% the CPU time
    plan_general();

    // ~10% the CPU time
    smoothPath(path, 0.9081, 0.2356, 0.0230, 0.6117, 0);

    if (INTERPOLATION_TYPE != 0 && path.size() > 2) {
        root_path = path;
        switch (INTERPOLATION_TYPE) {
            case 1:
                interpolateLinear(&path, NUMBER_OF_PATH_POINTS);
                break;
            case 2:
                interpolateAkimaSpline(path, GN_DISTANCE_BETWEEN_POINTS, NUMBER_OF_PATH_POINTS); // ~50% the CPU time
                break;
            case 3:
                interpolateCubicSpline(path, GN_DISTANCE_BETWEEN_POINTS, NUMBER_OF_PATH_POINTS);
                break;
            case 4:
                interpolateGaussNewton(path, GN_DISTANCE_BETWEEN_POINTS, NUMBER_OF_PATH_POINTS, GN_POLYNOMIAL_DEGREE, GN_MAX_ITER, GN_PRECISION);
                break;
            default:
                break;
        }
    }
    ROS_INFO("Path size: %lu\n", path.size());
    updateState();
}

void DiscoveryPathPlanner::publishRviz()
{
    // rvizMgr.addSmallOrangeCones(coneChain->getOrangeSmallCones());
    // rvizMgr.addBigOrangeCones(coneChain->getOrangeBigCones());
    // rvizMgr.addYellowCones(coneChain->getYellowCones());
    // rvizMgr.addBlueCones(coneChain->getBlueCones());
    // rvizMgr.addLeftConeChainOrder(coneChain->getLeftChain().getChain());
    // rvizMgr.addRightConeChainOrder(coneChain->getRightChain().getChain());
    // if (coneChain->getStartPoint())
    //     rvizMgr.addStart(*coneChain->getStartPoint());
    // rvizMgr.addTarget(target);
    //PathPlanner::interpolateLinear(path, NUMBER_OF_PATH_POINTS);
    rvizMgr.addPathPoints(path);
    rvizMgr.addRootPathPoints(root_path);
    rvizMgr.update();
}

// TODO
//  Actually implement
void DiscoveryPathPlanner::updateState()
{
    if (state == MissionState::LOOKING_FOR_START)
    {
        // coneChain->lookForStart();
        // if (coneChain->getStartPoint() != nullptr)
            state = MissionState::BEFORE_START;
    }
    else if (state == MissionState::BEFORE_START)
    {
        // if (isBehindVehicle(*coneChain->getStartPoint(), bolide) &&
        //     geometry::getDistance(*coneChain->getStartPoint(), bolide.getPosition()) > 5)
        // {
        //     state = MissionState::DURING_FINAL_LAP;
        // }
    }
    else if (state == MissionState::DURING_FINAL_LAP)
    {
        // look for the finish point (start)
        // if found send signal to control that finish point was found
    }
    else if (state == MissionState::APPROACHING_FINISH_POINT)
    {
        // slow down
    }
}