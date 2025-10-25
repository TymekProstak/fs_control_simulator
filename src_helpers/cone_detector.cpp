#include "cone_detector.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>  // std::move

namespace lem_dynamics_sim_ {

/// Zakładane struktury (dla kontekstu):
/// struct State { double x, y, yaw; };
/// struct Track_cone { double x, y, z, distance_raw, distance; std::string color; };
/// struct Track { std::vector<Track_cone> cones; };
/// struct ParamBank { double get(const std::string& key) const; };

//// ===========================================================================
//// 1) Transformacja toru globalnego do układu kamery
//// ===========================================================================

Track track_in_camera_frame(const State& state,
                            const Track& track_global,
                            const ParamBank& P)
{
    Track track_vehicle_frame;
    track_vehicle_frame.cones.reserve(track_global.cones.size());

    const double cos_yaw = std::cos(state.yaw);
    const double sin_yaw = std::sin(state.yaw);

    // Prefetch offsetów kamery
    const double x_cam = P.get("x_cog_to_camera");
    const double y_cam = P.get("y_cog_to_camera");
    const double z_cam = P.get("z_cog_to_camera");

    for (const auto& cone : track_global.cones)
    {
        const double dx = cone.x - state.x;
        const double dy = cone.y - state.y;

        Track_cone cv;
        cv.x = dx * cos_yaw + dy * sin_yaw + x_cam;
        cv.y = -dx * sin_yaw + dy * cos_yaw + y_cam;
        cv.z = cone.z + z_cam;
        cv.color = cone.color;

        const double dist = fast_sqrt(cv.x * cv.x + cv.y * cv.y + cv.z * cv.z);
     
        cv.distance = dist; // przed dodaniem szumu są równe

        track_vehicle_frame.cones.emplace_back(std::move(cv));
    }

    return track_vehicle_frame;
}

//// ===========================================================================
//// 2) Symulacja pojedynczego zdjęcia („shoot_a_frame”)
////    – filtracja stożków w FOV, model wykrycia, szum pozycji
//// ===========================================================================

Track shoot_a_frame(const Track& global_track, const ParamBank& P, const State& state)
{
    // --- 1. Transformacja do układu kamery
    Track local = track_in_camera_frame(state, global_track, P);

    // --- 2. Prefetch parametrów
    const double max_range    = P.get("max_vision_range");
    const double fov_W        = P.get("fov_W");
    const double fov_H        = P.get("fov_H");
    const double tan_W2       = std::tan(0.5 * fov_W);
    const double tan_H2       = std::tan(0.5 * fov_H);
    const double camera_range = P.get("camera_range");

    // Parametry błędu
    constexpr double a = 14.4;
    constexpr double b = 0.144;

    // Parametry t-Studenta
    constexpr double df = 6.0;
    const double std_corr = std::sqrt(df / (df - 2.0));

    // --- 3. RNG (thread_local, by uniknąć kosztów tworzenia)
    static thread_local std::mt19937 gen{ std::random_device{}() };
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);
    std::student_t_distribution<double>    t_dist(df);

    // --- 4. Wynikowy tor z widocznymi stożkami
    Track visible;
    visible.cones.reserve(local.cones.size());

    // --- 5. Główna pętla filtracji i symulacji błędu
    for (auto& c : local.cones)
    {
        // (a) Odrzuć stożki za kamerą
        if (c.x <= 0.0) continue;

        // (b) Odrzuć spoza zasięgu
        if (c.distance > max_range) continue;

        // (c) Odrzuć spoza pola widzenia
        if (std::abs(c.y) > tan_W2 * c.x) continue;
        if (std::abs(c.z) > tan_H2 * c.x) continue;

        // (d) Prawdopodobieństwo wykrycia
        const double pdet = std::exp(-c.distance / (1.5 * camera_range));
        if (uniform01(gen) >= pdet) continue;

        // (e) Szum pozycji (t-Student)
        const double RMSE   = a * fast_exp(b * c.distance);
        const double std_xy = RMSE * M_SQRT1_2;
        const double noise_x = std_xy * (t_dist(gen) / std_corr);
        const double noise_y = std_xy * (t_dist(gen) / std_corr);

        c.x += noise_x;
        c.y += noise_y;

        // dystancu nie aktualziujmy bo tak nie jest potem używany w pipelinie SLAM

        // (g) Zapis do wynikowego toru
        visible.cones.emplace_back(std::move(c));
    }

    return visible;
}

} // namespace lem_dynamics_sim_
