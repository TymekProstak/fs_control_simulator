#include "../../include/path_planner/TrackdrivePathPlanner.h"
#include "cone_chain/ConeLookup.h"
#include <common/config.h>

TrackdrivePathPlanner::TrackdrivePathPlanner() : PathPlanner()
{
    // coneChain = std::make_unique<AutocrossConeChainCreator>(AutocrossConeChainCreator());
}

// Sub-step method
void TrackdrivePathPlanner::plan_general() {

    ROS_INFO("Loaded cones\n");

    if (cl.GetLeftCones().size() <= 1 || cl.GetRightCones().size() <= 1) {
        ROS_WARN("Bad Cones!\n");
        return;
    }

    closed_loop = false;

    // save starting position for later (to check if is looping)
    Vec2 start_pos = bolide.getPosition();
    // Find cones closest to the car and start from them
    const Cone* leftCone = cl.LookupClosestLeftCone(start_pos);
    const Cone* rightCone = cl.LookupClosestRightCone(start_pos);
    const Cone* last_leftCone = nullptr, *last_rightCone = nullptr;

    const auto start_dir = Vec2((leftCone->getPosition().y - rightCone->getPosition().y), -(leftCone->getPosition().x - rightCone->getPosition().x)).normalized();

    //rightCone = cl.LookupClosestRightCone(leftCones[leftCone].getPosition()) - rightCones.data();

    Vec2 trackPoint = (leftCone->getPosition() + rightCone->getPosition()) / 2;
    const Vec2 start_track_point = trackPoint;

    int iter = 0;
    bool left_start = false;
    // Following a point in the center of the track find 2 closest cones (left and right),
    // add middle point to path, move the center point forward. Repeat
    while (iter < PATH_PLANNING_MAX_ITERATIONS) {

        //ROS_INFO("Path-Planning iteration %i\n", iter);

        trackPoint = (leftCone->getPosition() + rightCone->getPosition()) / 2;
        if (!path.empty() && (trackPoint - path.back()).length_squared() > (MAX_DISTANCE_BETWEEN_PATH_POINTS * MAX_DISTANCE_BETWEEN_PATH_POINTS))
            break;

        // Break early if looping
        if ((start_track_point - trackPoint).length_squared() < (POSITION_PROXIMITY_THRESHOLD * POSITION_PROXIMITY_THRESHOLD)) { // Distance check
            if (left_start && (start_pos - trackPoint).normalized().dot(start_dir) < -0.5) { // Alignment check (is the point behind)
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
        if (iter_1 >= 20 || new_leftCone == last_leftCone || new_rightCone == last_rightCone)
            break;

        // Assign only if changed
        last_leftCone = leftCone != new_leftCone ? leftCone : last_leftCone;
        last_rightCone = rightCone != new_rightCone ? rightCone : last_rightCone;

        leftCone = new_leftCone;
        rightCone = new_rightCone;

        iter++;
    }
}

// This is called right after getting a pose callback, to get the latest position
// For more detailed explanation please refer to: https://drive.google.com/file/d/1LiPxNrrZPKTRvOrwyxDWQOYxaG-hRztZ/view?usp=sharing
void TrackdrivePathPlanner::post_PoseCallbackCallback(const ros::TimerEvent &event) {
    static Vec2 last_bolide_pos {bolide.getPosition()};
    static bool was_last_ahead {false};
    Vec2 bolide_pos = bolide.getPosition();

    // Check if the bolide's position has changed
    if (last_bolide_pos == bolide_pos) return;

    if ((finish_center - bolide_pos).length_squared() > (LAP_COUNTING_FINISH_MIN_DIST * LAP_COUNTING_FINISH_MIN_DIST)) {
        last_bolide_pos = bolide_pos;
        return;
    }

    //ROS_INFO_THROTTLE(1.0, "Counting Laps, bolide pos: (%.2f, %.2f)\n", bolide_pos.x, bolide_pos.y);

    Vec2 v1 = (last_bolide_pos - bolide_pos).normalized();
    Vec2 across_finish_vec = finish_right - finish_left;
    Vec2 v2 = Vec2(across_finish_vec.y, -across_finish_vec.x).normalized();
    Vec2 v3 = (bolide_pos - finish_center).normalized();

    // ROS_INFO_THROTTLE(1.0, "v1 = (%.2f, %.2f), v2 = (%.2f, %.2f), v3 = (%.2f, %.2f)", v1.x, v1.y, v2.x, v2.y, v3.x, v3.y);
    // ROS_INFO_THROTTLE(1.0, "v1.dot(v2) = %.3f, v2.dot(v3) = %.3f, v3.dot(v1) = %.3f", v1.dot(v2), v2.dot(v3), v3.dot(v1));

    // Check if is ahead or behind
    if (v1.dot(v2) > LAP_COUNTING_ANGLE_THRESHOLD) {
        if (v2.dot(v3) > LAP_COUNTING_ANGLE_THRESHOLD
            && v3.dot(v1) > LAP_COUNTING_ANGLE_THRESHOLD) {
            // finish is ahead
            was_last_ahead = true;
        }
        else if (v2.dot(v3) < (-LAP_COUNTING_ANGLE_THRESHOLD)
            && v3.dot(v1) < (-LAP_COUNTING_ANGLE_THRESHOLD)) {
            // finish is behind
            if (was_last_ahead) {
                // Just crossed the finish line
                lap_count++;
                ROS_INFO("Lap count: %i\n", lap_count);
            }
            was_last_ahead = false;
        }
        else
            was_last_ahead = false;
    }

//early_return:
    last_bolide_pos = bolide_pos;
}

void TrackdrivePathPlanner::initialize_lap_counting() {
    lap_count = 0;

    //ROS_INFO("Initializing Lap Counting\n");

    // Calculate finish positions (used for lap counting)
    for (Cone lfCone : cl.GetLeftFinishCones())
        finish_left += lfCone.getPosition();
    finish_left = finish_left / cl.GetLeftFinishCones().size();

    for (Cone rfCone : cl.GetRightFinishCones())
        finish_right += rfCone.getPosition();
    finish_right = finish_right / cl.GetRightFinishCones().size();

    finish_center = (finish_left + finish_right) / 2;

    //ROS_INFO("finish center = (%.2f, %.2f)\n", finish_center.x, finish_center.y);
}

void TrackdrivePathPlanner::plan(pcl::PointCloud<pcl::PointXYZL> &cones, BolideDescriptor &bolide)
{
    if (cones.size() <= 1) {
        return;
    }

    cl.InsertCones(cones);

    plan_general();

    #ifdef PERFORM_PARAMETER_OPTIMIZATION
    if (!track_true_center.empty()) {
        // auto objective = [&](const std::vector<double> &params)->double{
        //     // original_path is the noisy input, target is the true center
        //     std::vector<Vec2> candidate = path;
        //     //smoothLowpassPreserveCorners(candidate, params[0], static_cast<int>(params[1]), params[2], params[3]);
        //     smoothPath(candidate, params[0], params[1], params[2], params[3], 0);
        //     // compute error:
        //     // - p = 4 to penalize larger deviations
        //     // - add max_weight to punish large local errors even heavier
        //     double err = computePathError(candidate, track_true_center,
        //                                   0.25f,           // sample spacing
        //                                   false,            // symmetric
        //                                   2.0f,            // p-norm exponent
        //                                   0.001,             // max weight multiplier (tweak)
        //                                   4.0f);           // exponent on max distance
        //     return err;
        // };

        class ArbitraryFunctionType
        {
            const std::vector<Vec2>& path;
            const std::vector<Vec2>& track_true_center;
        public:
            ArbitraryFunctionType(const std::vector<Vec2> & path, const std::vector<Vec2>& track_true_center) : path(path), track_true_center(track_true_center) {};
            // This should return f(x).
            double Evaluate(const arma::mat &x) const {
                if (x.min() < 0 || x[1] > 1 || x[3] > 1 || x[0] > 2 || x[2] > 2) return 1e2;
                // original_path is the noisy input, target is the true center
                std::vector<Vec2> candidate = path;
                //smoothLowpassPreserveCorners(candidate, params[0], static_cast<int>(params[1]), params[2], params[3]);
                smoothPath(candidate, x[0], x[1], x[2], x[3], 0);
                // compute error:
                // - p = 4 to penalize larger deviations
                // - add max_weight to punish large local errors even heavier
                double err = computePathError(candidate, track_true_center,
                                              0.25f, // sample spacing
                                              false, // symmetric
                                              2.0f, // p-norm exponent
                                              0.01, // max weight multiplier
                                              3.0f); // exponent on max distance
                return err;
            }
        };



        //std::vector<std::pair<double,double>> bounds = { {0.0, 1.0}, {1.0, 6.0}, {0.0, M_PI/2}, {0.0, M_PI/2}   }; // strength in [0,1], kernel radius in [1..6]
        //std::vector<std::pair<double,double>> bounds = { {0.0, 2.0}, {0.0, 1.0}, {0.0, 1.0}, {0.0, 1.0} }; // Simpler method
        // SimpleOptimizerOptions opt; opt.grid_steps = 7; opt.max_iters = 1000;
        // std::vector<double> best = simpleOptimize(objective, bounds, opt);

        printf("starting optimization!\n");

        //ens::SPSA opt(0.05, 0.0102, 0.1, 0.08, 100000, 1e-5);
        ens::CNE opt;
        arma::mat theta {0.9, 0.4, 0.1, 0.3};
        ArbitraryFunctionType af(path, track_true_center);
        const auto err = opt.Optimize(af, theta);
        printf("error: %f\n", err);

        theta.print("Best params: ");
        auto best = theta;

        // DEOptions deopts;
        // deopts.population_size = 60;
        // deopts.max_generations = 300;
        // deopts.F = 0.8;
        // deopts.CR = 0.85;
        // deopts.verbose = true;
        // deopts.stagnation_generations = 50;
        // deopts.tol = 1e-5;
        //
        // auto res = differential_evolution(objective, bounds, deopts);
        // std::vector<double> best = res.first;
        // double best_val = res.second;
        //
        // std::cout << "DE best: ";
        // for (double v : best) std::cout << v << ", ";
        // std::cout << " val=" << best_val << "\n";
        //
        // printf("Best params: %f, %f, %f, %f, error: %f\n", best[0], best[1], best[2], best[3], objective(best));

        //smoothLowpassPreserveCorners(path, best[0], static_cast<int>(best[1]), best[2], best[3]);
        smoothPath(path, best[0], best[1], best[2], best[3], 0);
    }
    else
#endif
        // Optim. results for some shitty ass, 10 meter long ass track: 0.3882   0.2662   0.6840   0.4641
        // also these from more curvy track: 0.9681   0.2356   0.0230   0.7117
        smoothPath(path, 0.9081, 0.2356, 0.0230, 0.6117, 0);

    initialize_lap_counting();

    updateState();
}

void TrackdrivePathPlanner::plan(const dv_interfaces::Cones &cones_msg, BolideDescriptor &bolide)
{
    if (cones_msg.cones.size() <= 1) {
        return;
    }

    cl.InsertCones(cones_msg);

    plan_general();

    #ifdef PERFORM_PARAMETER_OPTIMIZATION
    if (!track_true_center.empty()) {
        class ArbitraryFunctionType
        {
            const std::vector<Vec2>& path;
            const std::vector<Vec2>& track_true_center;
        public:
            ArbitraryFunctionType(const std::vector<Vec2> & path, const std::vector<Vec2>& track_true_center) : path(path), track_true_center(track_true_center) {};
            // This should return f(x).
            double Evaluate(const arma::mat &x) const {
                if (x.min() < 0 || x[1] > 1 || x[3] > 1 || x[0] > 2 || x[2] > 2) return 1e2;
                // original_path is the noisy input, target is the true center
                std::vector<Vec2> candidate = path;
                //smoothLowpassPreserveCorners(candidate, params[0], static_cast<int>(params[1]), params[2], params[3]);
                smoothPath(candidate, x[0], x[1], x[2], x[3], 0);
                // compute error:
                // - p = 4 to penalize larger deviations
                // - add max_weight to punish large local errors even heavier
                double err = computePathError(candidate, track_true_center,
                                              0.25f, // sample spacing
                                              false, // symmetric
                                              2.0f, // p-norm exponent
                                              0.01, // max weight multiplier
                                              3.0f); // exponent on max distance
                return err;
            }
        };


        printf("starting optimization!\n");

        //ens::SPSA opt(0.05, 0.0102, 0.1, 0.08, 100000, 1e-5);
        ens::CNE opt;
        arma::mat theta {0.9, 0.4, 0.1, 0.3};
        ArbitraryFunctionType af(path, track_true_center);
        const auto err = opt.Optimize(af, theta);
        printf("error: %f\n", err);

        theta.print("Best params: ");
        auto best = theta;
        smoothPath(path, best[0], best[1], best[2], best[3], 0);
    }
    else
#endif
        // Optim. results for some shitty ass, 10 meter long ass track: 0.3882   0.2662   0.6840   0.4641
        // also these from more curvy track: 0.9681   0.2356   0.0230   0.7117
        smoothPath(path, 0.9081, 0.2356, 0.0230, 0.6117, 0);

    initialize_lap_counting();

    updateState();
}

void TrackdrivePathPlanner::publishRviz()
{
    // rvizMgr.addSmallOrangeCones(coneChain->getOrangeSmallCones());
    // rvizMgr.addBigOrangeCones(coneChain->getOrangeBigCones());
    // rvizMgr.addYellowCones(coneChain->getYellowCones());
    // rvizMgr.addBlueCones(coneChain->getBlueCones());
    // rvizMgr.addLeftConeChainOrder(coneChain->getLeftChain().getChain());
    // rvizMgr.addRightConeChainOrder(coneChain->getRightChain().getChain());
    // rvizMgr.addTarget(target);
    rvizMgr.addPathPoints(path);
    rvizMgr.update();
}

// TODO
//  Actually implement
void TrackdrivePathPlanner::updateState()
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