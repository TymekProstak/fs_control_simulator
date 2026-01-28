#include "../include/Simulation.h"

#include <dv_interfaces/Cones.h>
#include <External/imgui_internal.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32MultiArray.h"

Simulation::Simulation(int seed) {
    if (!ros::isInitialized())
        return;

    gen = std::mt19937(seed);

    nh = new ros::NodeHandle("~");

    path_sub = nh->subscribe("/path_planning/path", 1, &Simulation::path_callback, this, ros::TransportHints().tcpNoDelay());
    path_planning_center_points.reserve(32);
    persistent_path_planning_center_points.reserve(32);

    path_makers_sub = nh->subscribe("/path_planning/markers", 1, &Simulation::path_markers_callback, this);

    cone_pub = nh->advertise<dv_interfaces::Cones>("/dv_cone_detector_fake/cones", 10);
    cone_persistent_marker_pub = nh->advertise<visualization_msgs::MarkerArray>("/viz/cones_gt", 1,true);

    track_true_center = nh->advertise<std_msgs::Float32MultiArray>("/path_planning/track_true_center", 1, true);

    car_direction_euler.x = 1;
    car_direction.setEuler(0, 0, M_PI);
}

void Simulation::Recalculate_Bezier() {
    //bezier_control_points.clear();

    int n = center_points.size() - 1;
    if (n < 1) {
        bezier_control_points.clear();
        return;
    }

    if (n == 1) {
        bezier_control_points.resize(2);
        // Special case: Bezier curve should be a straight line.

        // 3P1 = 2P0 + P3
        bezier_control_points[0][0] = (2 * center_points[0] + center_points[1]) / 3;

        // P2 = 2P1 – P0
        bezier_control_points[0][1] = 2 * bezier_control_points[0][0] - center_points[0];

        // 3P1 = 2P0 + P3
        bezier_control_points[1][0] = (2 * center_points[0] + center_points[1]) / 3;

        // P2 = 2P1 – P0
        bezier_control_points[1][1] = 2 * bezier_control_points[0][0] - center_points[0];

        return;
    }

    if (close_loop) {
        for (int i = 0; i < std::min(n, BEZIER_MAX_CURVE_LOOP_BACKTRACK); i++) {
            center_points.emplace_back(center_points[i]);
        }
        // center_points.emplace_back(center_points[0]);
        // center_points.emplace_back(center_points[1]);
    }
    else
        center_points.emplace_back(2*center_points[n] - center_points[n-1]); // Push dummy point at the end

    if (close_loop) {
        for (int i = 0; i < std::min(n, BEZIER_MAX_CURVE_LOOP_BACKTRACK); i++) {
            center_points.emplace_front(center_points[n]);
        }
        // center_points.emplace_front(center_points[n]); // Add the end point to the front to close it
        // center_points.emplace_front(center_points[n-1]); // Add the end point to the front to close it
    }

    int original_n = n;
    n += close_loop ? (BEZIER_MAX_CURVE_LOOP_BACKTRACK * 2) : 1;
    bezier_control_points.resize(n);

    // Calculate first Bezier control points
    // Right hand side vector
    Vec2 *rhs = new Vec2[n];

    // Set right hand side X values
    for (int i = 1; i < n - 1; ++i)
        rhs[i] = 4 * center_points[i] + 2 * center_points[i + 1];
    rhs[0] = center_points[0] + 2 * center_points[1];
    rhs[n - 1] = (8 * center_points[n - 1] + center_points[n]) / 2.0;
    // Get first control points

    Vec2 *x = new Vec2[n]; // Solution vector.
    Vec2 *tmp = new Vec2[n]; // Temp workspace.

    Vec2 b = {2.0, 2.0};
    x[0] = rhs[0] / b;
    for (int i = 1; i < n; i++) // Decomposition and forward substitution.
    {
        tmp[i] = Vec2(1, 1) / b;
        b = (i < n - 1 ? Vec2(4, 4) : Vec2(3.5, 3.5)) - tmp[i];
        x[i] = (rhs[i] - x[i - 1]) / b;
    }
    for (int i = 1; i < n; i++)
        x[n - i - 1] -= tmp[n - i] * x[n - i]; // Backsubstitution.


    for (int i = 0; i < n; ++i) {
        // Swap manually set elements before they get reassigned (accounting for the backtrack size)
        if (i < n - BEZIER_MAX_CURVE_LOOP_BACKTRACK && center_points[i+BEZIER_MAX_CURVE_LOOP_BACKTRACK].manual_setup) {
            bezier_control_points[i+BEZIER_MAX_CURVE_LOOP_BACKTRACK][0] = bezier_control_points[i][0];
            bezier_control_points[i+BEZIER_MAX_CURVE_LOOP_BACKTRACK][1] = bezier_control_points[i][1];
        }
        if (center_points[i].manual_setup)
            continue;

        // First control point
        bezier_control_points[i][0] = x[i];
        // Second control point
        if (i < n - 1) {
            bezier_control_points[i][1] = 2 * center_points[i + 1] - x[i + 1];
        }
        else
            bezier_control_points[i][1] = center_points[n] + x[n - 1] / 2;
    }

    center_points.pop_back(); // Pop the back dummy
    //bezier_control_points.pop_back(); // Save (dont pop) one bezier control point of case of closed loop

    if (close_loop) {
        for (int i = 1; i < std::min(original_n, BEZIER_MAX_CURVE_LOOP_BACKTRACK); i++) {
            center_points.pop_back();
            center_points.pop_front();
            bezier_control_points.pop_back();
            bezier_control_points.pop_front();
        }
        // center_points.pop_back(); // Pop the loop back dummy
        // center_points.pop_front(); // Pop the loop front dummy
        center_points.pop_front(); // Pop the loop front dummy

        // bezier_control_points.pop_back();
        // bezier_control_points.pop_front();
        bezier_control_points.pop_front();
    }

    Recalculate_Cones();

    delete[] rhs;
    delete[] x;
    delete[] tmp;
}

void Simulation::path_callback(const geometry_msgs::PoseArray &msg) {
    path_planning_center_points.clear();
    //path_planning_center_points.resize(msg.poses.size());

    geometry_msgs::PoseStamped pathpose;
    pathpose.header = msg.header;

    for (int i = 0; i < msg.poses.size(); i++) {
        pathpose.pose = msg.poses[i];
        //path_planning_center_points[i] = Vec2(static_cast<float>(pathpose.pose.position.x), static_cast<float>(pathpose.pose.position.y));
        path_planning_center_points.push_back(Vec2(static_cast<float>(pathpose.pose.position.x), static_cast<float>(pathpose.pose.position.y)) / SIM_SCALE_FACTOR);
        persistent_path_planning_center_points.emplace_back(path_planning_center_points.back());
        //printf("path: (%.2f, %.2f)\n", pathpose.pose.position.x, pathpose.pose.position.y);
    }

    //printf("path_planning_center_points.size() = %ld\n", path_planning_center_points.size());
}

void Simulation::path_markers_callback(const visualization_msgs::MarkerArray &msg) {
    nice_path_planning_markers.clear();
    //printf("Got markers callback, size: %ld\n", msg.markers.size());
    if (msg.markers.empty())
        return;

    for (int i = 0; i < msg.markers.size(); i++) {
        auto marker = msg.markers[i];
        marker.pose.position.x /= SIM_SCALE_FACTOR;
        marker.pose.position.y /= SIM_SCALE_FACTOR;
        marker.pose.position.z /= SIM_SCALE_FACTOR;
        nice_path_planning_markers.emplace_back(marker);
    }
}

void Simulation::NextPosition_PathPlanning(int step) {
    if (persistent_path_planning_center_points.empty() || (step > 0 && car_track_point >= persistent_path_planning_center_points.size()-1)
        || (step < 0 && car_track_point <= 0)) {
        return;
    }

    car_track_point += step;

    car_position = persistent_path_planning_center_points[car_track_point];

    Vec2 dir = GetRotationAtTrack_PathPlanning(car_track_point);
    dir /= dir.length();
    car_direction_euler = dir;
    const float theta = atan2(dir.y, dir.x);
    //car_direction.setEuler(0, 0, theta);

    car_direction.setW(cos(theta / 2));
    car_direction.setZ(sin(theta / 2));
    // car_direction.w = cos(theta);
    // car_direction.y = sin(theta);

    if (car_track_point >= persistent_path_planning_center_points.size() - 1) {
        UpdatePathPlanning();
        return;
    }

}

void Simulation::UpdateCarPosition() {
    //ros::Duration(0.2).sleep();

    geometry_msgs::TransformStamped tf_transform;
    tf_transform.header.stamp = ros::Time::now();
    tf_transform.header.frame_id = "map"; // Parent frame
    tf_transform.child_frame_id = "bolide_CoG"; // Child frame
    tf_transform.transform.translation.x = car_position.x * SIM_SCALE_FACTOR;
    tf_transform.transform.translation.y = car_position.y * SIM_SCALE_FACTOR;
    tf_transform.transform.translation.z = 0;
    tf_transform.transform.rotation.x = car_direction.getX();
    tf_transform.transform.rotation.y = car_direction.getY();
    tf_transform.transform.rotation.z = car_direction.getZ();
    tf_transform.transform.rotation.w = car_direction.getW();
    tf_broadcaster.sendTransform(tf_transform);
}

int Simulation::GetClosestPointOnTheTrack(Vec2 point, float threshold) const {
    const size_t n = center_points.size();

    if (n < 2)
        return -1;

    Vec2 last_point = close_loop ? center_points.back() : center_points[0];
    //Vec2 last_point = center_points[0];
    //Vec2 next_point = close_loop ? center_points[1] : center_points[1];

    threshold = threshold * threshold; // square threshold because we use (x*x)+(y*y) for length in Vec2 (no sqrt) - It's blazing fast!

    if (smooth_curve) {
        for (int i = (close_loop ? 0 : 1); i < n ; i++) {
            //last_point = center_points[(i+n-1)%n];
            Vec2 dir = center_points[i] - last_point;
            auto len = dir.length();

            float stride = cones_min_distance/len;
            for (float f = stride; f <= 1+stride; f += stride) {
                Vec2 pos;
                if (i == 0)
                    pos = ImBezierCubicCalc(last_point, bezier_control_points[n - 1][0], bezier_control_points[n - 1][1],
                                            center_points[i], f);
                else
                    pos = ImBezierCubicCalc(last_point, bezier_control_points[i - 1][0], bezier_control_points[i - 1][1],
                                             center_points[i], f);
                if ((pos - point).len_square() < threshold) {
                    return i;
                }
            }

            last_point = center_points[i];
        }
    }
    else {
        for (int i = (close_loop ? 0 : 1); i < n; i++) {
            //last_point = center_points[(i+n-1)%n];
            Vec2 dir = last_point - center_points[i%n];
            auto len = dir.length();

            float stride = cones_min_distance/len;
            for (float f = 0; f <= 1; f += stride) {
                Vec2 pos = ImLerp(center_points[i%n], last_point, 1-f);

                if ((pos - point).len_square() < threshold) {
                    return i;
                }
            }

            last_point = center_points[i];
        }
    }

    return -1;
}

Vec2 Simulation::GetRotationAtTrack_PathPlanning(int point) const {
    const size_t n = persistent_path_planning_center_points.size();

    if (n < 2)
        return {0, 0};

    if (point >= n - 1) {
        Vec2 prev_point = persistent_path_planning_center_points[point-1];
        Vec2 current_point = persistent_path_planning_center_points[point];
        Vec2 direction = (current_point - prev_point);
        return direction;
    }

    Vec2 next_point = persistent_path_planning_center_points[point+1];
    Vec2 current_point = persistent_path_planning_center_points[point];
    Vec2 direction = (next_point - current_point);

    return direction;
}


bool Simulation::AddCones(Vec2 pos, Vec2 dir, std::array<Vec2, 2> control_points, bool force) {
    // 2 vectors rotated by 90 deg.
    Vec2 p1 = pos + Vec2((-TRACK_DIRECTION) * dir.y, (TRACK_DIRECTION) * dir.x);
    Vec2 p2 = pos + Vec2((TRACK_DIRECTION) * dir.y, (-TRACK_DIRECTION) * dir.x);

    bool is_p1_good = true, is_p2_good = true;

    if (!force && !isnan(control_points[0].x) && !isnan(control_points[1].y)) {
        for (auto& cp : control_points) {
            if ((p1 - cp).length() < cones_padding/2) {
                is_p1_good = false;
            }
            if ((p2 - cp).length() < cones_padding/2) {
                is_p2_good = false;
            }
        }
    }

    if (is_p1_good && !force)
        for (auto& p : temp_cones_L) {
            if ((p-p1).length() < cones_max_distance) {
                is_p1_good = false;
                break;
            }
        }

    if (is_p2_good && !force)
        for (auto& p : temp_cones_R) {
            if ((p-p2).length() < cones_max_distance) {
                is_p2_good = false;
                break;
            }
        }

    if (is_p1_good)
        temp_cones_L.emplace_front(p1);
    if (is_p2_good)
        temp_cones_R.emplace_front(p2);

    // Kick some of the cones
    if (dist(gen) < cones_kicked_percentage)
        p1 += Vec2(dir.y, -dir.x) * (dist(gen) * cones_kick_strength);
    if (dist(gen) < cones_kicked_percentage)
        p2 += Vec2(-dir.y, dir.x) * (dist(gen) * cones_kick_strength);

    if (is_p1_good)
        cones_left_positions.emplace_back(p1);
    if (is_p2_good)
        cones_right_positions.emplace_back(p2);

    if (temp_cones_L.size() > CONES_TEMP_BUFFER_SIZE)
        temp_cones_L.pop_back();

    if (temp_cones_R.size() > CONES_TEMP_BUFFER_SIZE)
        temp_cones_R.pop_back();

    return is_p1_good || is_p2_good;
}

void Simulation::Recalculate_Cones() {
    cones_left_positions.clear();
    cones_right_positions.clear();
    temp_cones_L.clear(); temp_cones_R.clear();

    const size_t n = center_points.size();

    if (n < 2)
        return;

    //Vec2 last_point = close_loop ? center_points.back() : center_points[0];
    //Vec2 next_point = close_loop ? center_points[1] : center_points[1];

    if (smooth_curve) {
        for (int i = 1; i < n + (close_loop ? 1 : 0); i++) {
            const Vec2 last_point = center_points[(i+n-1)%n];
            Vec2 dir = center_points[i%n] - last_point;
            auto len = dir.length();

            Vec2 first_point_added = Vec2(NAN,0);
            Vec2 second_point_added = Vec2(NAN,0);
            int added_cones = 0;
            float stride = cones_min_distance/len;
            Vec2 last_bezier_point = last_point;
            for (float f = stride; f <= 1+stride; f += stride * get_randon_in_range(cones_rand_distance)) {
                Vec2 pos = ImBezierCubicCalc(last_point, bezier_control_points[(i + n - 1) % n][0],
                                             bezier_control_points[(i + n - 1) % n][1], center_points[i % n], f);

                dir = last_bezier_point - pos;
                len = dir.length();
                dir = dir / len * (cones_padding * get_randon_around_range(cones_rand_padding));

                if (AddCones(pos, dir, {last_point, center_points[i%n]})) {
                    if (isnan(first_point_added.x))
                        first_point_added = pos;
                    else if (isnan(second_point_added.x))
                        second_point_added = pos;
                    added_cones++;
                }
                last_bezier_point = pos;
            }

            // Insert orange cones
            if (i == 1 && added_cones >= 2 && !isnan(first_point_added.x) && !isnan(second_point_added.x)) {
                Vec2 track_center_back = (second_point_added * 2 + first_point_added) / 3;
                Vec2 track_center_front = (second_point_added + first_point_added * 2) / 3;

                dir = track_center_back - track_center_front;
                len = dir.length();
                dir = dir / len * (cones_padding * get_randon_around_range(cones_rand_padding));

                AddCones(track_center_back, dir, {last_point, center_points[i%n]}, true);
                AddCones(track_center_front, dir, {last_point, center_points[i%n]}, true);

                std::swap(cones_left_positions[1], *(cones_left_positions.end() - 1));
                std::swap(cones_left_positions[2], *(cones_left_positions.end() - 2));
                std::swap(cones_right_positions[1], *(cones_right_positions.end() - 1));
                std::swap(cones_right_positions[2], *(cones_right_positions.end() - 2));
            }

            //last_point = center_points[i];
        }
    }
    else {
        for (int i = 1; i < n + (close_loop ? 1 : 0); i++) {
            const Vec2 last_point = center_points[(i+n-1)%n];
            Vec2 dir = last_point - center_points[i%n];
            auto len = dir.length();
            dir = dir / len * (cones_padding * get_randon_around_range(cones_rand_padding));

            Vec2 first_point_added = Vec2(NAN,0);
            Vec2 second_point_added = Vec2(NAN,0);
            int added_cones = 0;
            float stride = cones_min_distance/len;
            for (float f = 0; f <= 1; f += stride * get_randon_in_range(cones_rand_distance)) {
                Vec2 pos = ImLerp(center_points[i%n], last_point, 1-f);

                if (AddCones(pos, dir, {last_point, center_points[i%n]})) {
                    if (isnan(first_point_added.x))
                        first_point_added = pos;
                    else if (isnan(second_point_added.x))
                        second_point_added = pos;
                    added_cones++;
                }
            }

            // Insert orange cones
            if (i == 1 && added_cones >= 2 && !isnan(first_point_added.x) && !isnan(second_point_added.x)) {
                Vec2 track_center_back = (second_point_added * 2 + first_point_added) / 3;
                Vec2 track_center_front = (second_point_added + first_point_added * 2) / 3;

                dir = track_center_back - track_center_front;
                len = dir.length();
                dir = dir / len * (cones_padding * get_randon_around_range(cones_rand_padding));

                AddCones(track_center_back, dir, {last_point, center_points[i%n]}, true);
                AddCones(track_center_front, dir, {last_point, center_points[i%n]}, true);

                std::swap(cones_left_positions[1], *(cones_left_positions.end() - 1));
                std::swap(cones_left_positions[2], *(cones_left_positions.end() - 2));
                std::swap(cones_right_positions[1], *(cones_right_positions.end() - 1));
                std::swap(cones_right_positions[2], *(cones_right_positions.end() - 2));
            }

            //last_point = center_points[i];
        }
    }
}

void Simulation::PublishCones(bool use_shuffle, int max_cones) const {
    dv_interfaces::Cones conesArray;
    conesArray.header.stamp = ros::Time::now();
    conesArray.header.frame_id = "map";
    max_cones = std::min(max_cones, static_cast<int>(std::max(cones_left_positions.size(), cones_right_positions.size())));
    int N = max_cones == -1 ? std::max(cones_left_positions.size(), cones_right_positions.size()) : max_cones;
    conesArray.cones.reserve(max_cones == -1 ? (cones_left_positions.size() + cones_right_positions.size()) : N);

    std::vector<int> shuffled_idx;
    if (use_shuffle) {
        shuffled_idx = std::vector<int> (N);
        std::iota(shuffled_idx.begin(), shuffled_idx.end(), 0);
        std::shuffle(shuffled_idx.begin(), shuffled_idx.end(), std::random_device());
    }

    for (int j = 0; j < N; j++)
    {
        int i;
        if (use_shuffle)
            i = shuffled_idx[j];
        else
            i = j;

        if (i < cones_left_positions.size()) {
            const Vec2 point = cones_left_positions[i];
            dv_interfaces::Cone cone;
            cone.x = point.x * SIM_SCALE_FACTOR;
            cone.y = point.y * SIM_SCALE_FACTOR;
            cone.z = 1;
            cone.confidence = 0.9;
            // if (i == 1)
            //     cone.class_name = "orange_small";
            // else
            cone.class_name = (i != 0 && i <= 2) ? "orange_big" : "blue";
            conesArray.cones.push_back(cone);
        }
        if (i < cones_right_positions.size()) {
            const Vec2 point = cones_right_positions[i];
            dv_interfaces::Cone cone;
            cone.x = point.x * SIM_SCALE_FACTOR;
            cone.y = point.y * SIM_SCALE_FACTOR;
            cone.z = 1;
            cone.confidence = 0.9;
            // if (i == 1)
            //     cone.class_name = "orange_small";
            // else
                //cone.class_name = "yellow";
            cone.class_name = (i != 0 && i <= 2) ? "orange_big" : "yellow";
            conesArray.cones.push_back(cone);
        }
    }

    // if (center_points.size() > 0) {
    //     // Fake big orange cone
    //     dv_interfaces::Cone cone;
    //     cone.x = (cones_right_positions.front().x - 8) * SIM_SCALE_FACTOR;
    //     cone.y = (cones_right_positions.front().y + 7) * SIM_SCALE_FACTOR;
    //     cone.z = 1;
    //     cone.confidence = 0.9;
    //     cone.class_name = "orange_big";
    //     conesArray.cones.push_back(cone);
    // }

    //printf("Sent cones of size %ld\n", conesArray.cones.size());
    cone_pub.publish(conesArray);

    PublishPersistentMarkerCones();
}

void Simulation::UpdatePathPlanning() {
    dv_interfaces::Cones conesArray;
    conesArray.header.stamp = ros::Time::now();
    conesArray.header.frame_id = "map";
    cone_pub.publish(conesArray);

    ros::Duration(0.3).sleep(); // 300ms delay before updating the car pos, so PathPlanning can get it's crap together

    UpdateCarPosition();
}

void Simulation::ResetPathPlanningMarkers() {
    nice_path_planning_markers.clear();
    path_planning_center_points.clear();
    persistent_path_planning_center_points.clear();

    car_track_point = 0;
}

std_msgs::ColorRGBA color_from_class_vision(const std::string &cls, float alpha = 0.95f) {
    std_msgs::ColorRGBA c;
    c.a = alpha;

    if (cls == "yellow") {
        c.r = 1.0f;
        c.g = 1.0f;
        c.b = 0.0f;
    }
    else if (cls == "blue") {
        c.r = 0.0f;
        c.g = 0.3f;
        c.b = 1.0f;
    }
    else if (cls == "orange") {
        c.r = 1.0f;
        c.g = 0.55f;
        c.b = 0.0f;
    }
    else {
        c.r = 0.7f;
        c.g = 0.7f;
        c.b = 0.7f;
    }

    return c;
}

void Simulation::PublishPersistentMarkerCones() const {
    visualization_msgs::MarkerArray arr;

    if (!cone_persistent_marker_pub) {
        ROS_ERROR("cone_persistent_marker_pub invalid; not publishing persistent markers");
        return;
    }

    visualization_msgs::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = ros::Time::now();
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    // Forever
    const ros::Duration kForever(0.0);

    int N = std::max(cones_left_positions.size(), cones_right_positions.size());
    int idx = 0;
    for (int i = 0; i < N; i++) {
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "cones_gt";
        m.type = visualization_msgs::Marker::CYLINDER;
        m.action = visualization_msgs::Marker::ADD;

        m.scale.x = 0.30;
        m.scale.y = 0.30;
        m.scale.z = 0.30;

        m.lifetime = kForever;
        m.frame_locked = false;

        if (i < cones_left_positions.size()) {
            const Vec2 point = cones_left_positions[i];
            // pozycja w układzie kamery
            m.pose.position.x = point.x;
            m.pose.position.y = point.y;
            m.pose.position.z = 0.15;
            m.pose.orientation.w = 1.0;
            m.id = idx++;

            std_msgs::ColorRGBA col = color_from_class_vision(i == 0 ? "orange" : "blue");
            m.color = col;
            arr.markers.push_back(m);
        }
        if (i < cones_right_positions.size()) {
            const Vec2 point = cones_right_positions[i];
            // pozycja w układzie kamery
            m.pose.position.x = point.x;
            m.pose.position.y = point.y;
            m.pose.position.z = 0.15;
            m.pose.orientation.w = 1.0;
            m.id = idx++;

            std_msgs::ColorRGBA col = color_from_class_vision(i == 0 ? "orange" : "yellow");
            m.color = col;
            arr.markers.push_back(m);
        }
    }

    cone_persistent_marker_pub.publish(arr);
}

void Simulation::PublishTrackTrueCenter() const {
    const size_t n = center_points.size();

    if (n < 2)
        return;

    std::vector<Vec2> fine_track_center_points;

    if (smooth_curve) {
        for (int i = 1; i < n + (close_loop ? 1 : 0); i++) {
            const Vec2 last_point = center_points[(i + n - 1) % n];
            Vec2 dir = center_points[i % n] - last_point;
            auto len = dir.length();

            float stride = cones_min_distance / len;
            for (float f = stride; f <= 1 + stride; f += stride ) {
                Vec2 pos = ImBezierCubicCalc(last_point, bezier_control_points[(i + n - 1) % n][0],
                                             bezier_control_points[(i + n - 1) % n][1], center_points[i % n], f);
                fine_track_center_points.push_back(pos);
            }
        }
    } else {
        for (int i = 1; i < n + (close_loop ? 1 : 0); i++) {
            const Vec2 last_point = center_points[(i + n - 1) % n];
            Vec2 dir = last_point - center_points[i % n];
            auto len = dir.length();

            float stride = cones_min_distance / len;
            for (float f = 0; f <= 1; f += stride) {
                Vec2 pos = ImLerp(center_points[i % n], last_point, 1 - f);
                fine_track_center_points.push_back(pos);
            }
        }
    }


    // Send an array of Vec2 as an array of floats (don't blame me, the times are rough)
    std_msgs::Float32MultiArray track_center_points_msg;
    track_center_points_msg.data.resize(fine_track_center_points.size() * 2);
    for (size_t i = 0; i < fine_track_center_points.size(); i++) {
        track_center_points_msg.data[i * 2] = fine_track_center_points[i].x;
        track_center_points_msg.data[i * 2 + 1] = fine_track_center_points[i].y;
    }
    track_true_center.publish(track_center_points_msg);
}

// void Simulation::PublishMarkerConesVis(int max_cones) const { // TODO REMOVE
//     if (!cone_marker_pub) {
//         ROS_ERROR("pub_markers_cones_vis_ invalid; not publishing vision markers");
//         return;
//     }
//
//     visualization_msgs::MarkerArray arr;
//
//     visualization_msgs::Marker del;
//     del.header.frame_id = "camera_base";
//     del.header.stamp = ros::Time::now();
//     del.action = visualization_msgs::Marker::DELETEALL;
//     arr.markers.push_back(del);
//
//     const ros::Duration lifetime(0);
//     int N = max_cones == -1 ? std::max(cones_left_positions.size(), cones_right_positions.size()) : max_cones;
//     int idx = 0;
//     for (int i = 0; i < N; i++) {
//         visualization_msgs::Marker m;
//         m.header.frame_id = "camera_base";
//         m.header.stamp = ros::Time::now();
//         m.ns = "cones_vis";
//         m.type = visualization_msgs::Marker::CUBE;
//         m.action = visualization_msgs::Marker::ADD;
//
//         // rozmiar: 0.3 m × 0.3 m × 0.3 m
//         m.scale.x = 0.30;
//         m.scale.y = 0.30;
//         m.scale.z = 0.30;
//
//         m.lifetime = lifetime;
//         m.frame_locked = false;
//
//         if (i < cones_left_positions.size()) {
//             const Vec2 point = cones_left_positions[i];
//             // pozycja w układzie kamery
//             m.pose.position.x = point.x;
//             m.pose.position.y = point.y;
//             m.pose.position.z = 0.15;
//             m.pose.orientation.w = 1.0;
//             m.id = idx++;
//
//             std_msgs::ColorRGBA col = color_from_class_vision(i == 0 ? "orange" : "blue");
//             m.color = col;
//             arr.markers.push_back(m);
//         }
//         if (i < cones_right_positions.size()) {
//             const Vec2 point = cones_right_positions[i];
//             // pozycja w układzie kamery
//             m.pose.position.x = point.x;
//             m.pose.position.y = point.y;
//             m.pose.position.z = 0.15;
//             m.pose.orientation.w = 1.0;
//             m.id = idx++;
//
//             std_msgs::ColorRGBA col = color_from_class_vision(i == 0 ? "orange" : "yellow");
//             m.color = col;
//             arr.markers.push_back(m);
//         }
//     }
//
//     cone_marker_pub.publish(arr);
// }
