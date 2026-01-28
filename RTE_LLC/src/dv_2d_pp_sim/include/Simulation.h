#ifndef SIMULATION_H
#define SIMULATION_H


#include <deque>
#include <common/Vec2.h>
#include <ros/ros.h>
#include <random>
#include <dv_interfaces/Cones.h>
#include <External/imgui.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define BEZIER_MAX_CURVE_LOOP_BACKTRACK 3
//#define CONES_PROXIMITY_REMOVE_THRESHOLD 100
#define CONES_TEMP_BUFFER_SIZE 12 // Amount of cones to compare to (proximity wise) when adding new one (AddCones())
#define SIM_SCALE_FACTOR 1
#define TRACK_DIRECTION (-1) // -1 for CW, 1 for CCW

inline ros::NodeHandle* nh;

class Simulation {
public:
    class Ex_Vec2 : public Vec2 {
        public:
        Ex_Vec2(float x, float y) : Vec2(x, y) {}
        Ex_Vec2(Vec2 v) : Vec2(v) {}
        Ex_Vec2& operator=(const Vec2& other) {this->x = other.x; this->y = other.y; return *this;}

        bool manual_setup = false;
    };

    class PP_Marker {
        public:
        Vec2 pos;
        std::string _text;
        float size;
        ImColor _color = ImColor(100, 100, 100, 255);

        PP_Marker(const visualization_msgs::Marker &marker) {
            _text = marker.text;
            pos = Vec2(marker.pose.position.x, marker.pose.position.y);
            size = (marker.scale.x + marker.scale.y); // Average * 2
            _color = ImColor(_clamp(marker.color.r, 0, 1), _clamp(marker.color.g, 0, 1), _clamp(marker.color.b, 0, 1), _clamp(marker.color.a, 0, 1));
        }
    };

    template <class ContainerAllocator>
    struct TrackPoints {
        std::vector<Vec2> points;
    };

    bool close_loop = false;
    bool smooth_curve = true;
    bool auto_update_car_pos = false;

    float cones_min_distance = 2;
    float cones_padding = 3;
    float cones_max_distance = 4;
    float cones_kicked_percentage = 0.0; // percentage of "hit" cones (outside the track)
    float cones_kick_strength = 1.0; // percentage of "hit" cones (outside the track)

    float cones_rand_padding = 0;
    float cones_rand_distance = 0;

    Vec2 car_position {0,0};
    tf2::Quaternion car_direction;
    Vec2 car_direction_euler;

    std::deque<Ex_Vec2> center_points{}; // Bezier main points

    std::deque<std::array<Vec2, 2>> bezier_control_points{};

    std::vector<Vec2> cones_left_positions{};
    std::vector<Vec2> cones_right_positions{};

    std::vector<Vec2> path_planning_center_points{}; // Last "batch" of path
    std::vector<Vec2> persistent_path_planning_center_points{}; // Accumulating points

    std::vector<PP_Marker> nice_path_planning_markers; // Cleaner way of storing markers optimized for GUI

    Simulation() = default;

    // Set up a constant seed for testing
    explicit Simulation(int seed);

    void Recalculate_Bezier();
    void Recalculate_Cones();

    void PublishCones(bool use_shuffle = false, int max_cones = -1) const;
    void PublishPersistentMarkerCones() const;
    void PublishTrackTrueCenter() const;
    //void PublishMarkerConesVis(int max_cones = -1) const; // TODO REMOVE
    void UpdatePathPlanning(); // Sends empty cones to PathPlanning
    void ResetPathPlanningMarkers();

    Vec2 GetRotationAtTrack_PathPlanning(int point) const;
    void NextPosition_PathPlanning(int step = 1);
    void UpdateCarPosition();

    int GetClosestPointOnTheTrack(Vec2 point, float threshold = 2) const;

    private:
    std::deque<Vec2> temp_cones_L;
    std::deque<Vec2> temp_cones_R;

    std::mt19937 gen;
    std::uniform_real_distribution<float> dist;

    //ros::NodeHandle* nh = nullptr;
    ros::Publisher cone_pub;
    ros::Publisher cone_persistent_marker_pub;
    ros::Publisher track_true_center;
    ros::Subscriber path_sub;
    ros::Subscriber path_makers_sub;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    int car_track_point = 0;

    void path_callback(const geometry_msgs::PoseArray& msg);
    void path_markers_callback(const visualization_msgs::MarkerArray& msg);

    float get_randon_in_range(const float range) {
        return 1 - dist(gen) * range;
    }

    // Centered around 1. Returns range (1, 1) for range = 0 and range (0, 2) for range = 1
    float get_randon_around_range(const float range) {
        return (1 - range) + (2 * range * dist(gen));
    }

    bool AddCones(Vec2 pos, Vec2 dir, std::array<Vec2, 2> control_points = {Vec2{NAN,NAN}, Vec2{NAN,NAN}},
                  bool force = false);
};

#endif //SIMULATION_H
