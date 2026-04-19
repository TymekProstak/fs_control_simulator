#include "cone_detector.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>  // std::move

namespace lem_dynamics_sim_ {

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
    const double x_cam = P.get("x_camera_to_cog");
    const double y_cam = P.get("y_camera_to_cog");
    const double z_cam = P.get("z_camera_to_cog");

    for (const auto& cone : track_global.cones)
    {
        const double dx = cone.x - state.x;
        const double dy = cone.y - state.y;

        Track_cone cv;
        cv.x = dx * cos_yaw + dy * sin_yaw + x_cam;
        cv.y = -dx * sin_yaw + dy * cos_yaw + y_cam;
        cv.z = cone.z + z_cam;
        cv.color = cone.color;

        const double dist = std::sqrt(cv.x * cv.x + cv.y * cv.y + cv.z * cv.z);
        cv.distance = dist;

        track_vehicle_frame.cones.emplace_back(std::move(cv));
    }

    return track_vehicle_frame;
}

//// ===========================================================================
//// 2) Symulacja pojedynczego zdjęcia („shoot_a_frame”)
////    – filtracja stożków w FOV, model wykrycia, szum pozycji (Gauss)
//// ===========================================================================

Track shoot_a_frame(const Track& global_track, const ParamBank& P, const State& state)
{
    // --- 1. Transformacja do układu kamery
    Track local = track_in_camera_frame(state, global_track, P);

    // --- 2. Prefetch parametrów
    const double max_range    = P.get("camera_range");
    const double fov_W        = P.get("fov_x") * M_PI / 180.0;
    const double fov_H        = P.get("fov_y") * M_PI / 180.0;
    const double tan_W2       = std::tan(0.5 * fov_W);
    const double tan_H2       = std::tan(0.5 * fov_H);
    const double camera_range = P.get("camera_range");

    // Parametry błędu (zależność RMSE od odległości)
    const  double a = P.get("vision_noise_a");
    const double b = P.get("vision_noise_b");

    // --- 3. RNG (thread_local, by uniknąć kosztów tworzenia)
    static thread_local std::mt19937 gen{ std::random_device{}() };
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);
    std::normal_distribution<double> normal(0.0, 1.0);  // Gauss(0,1)

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
        // const double pdet = std::exp(-c.distance / (1.5*camera_range));
        // if (uniform01(gen) >= pdet) continue;

        // (e) Szum pozycji (czysty Gauss)
        const double RMSE   = a * std::exp(b * c.distance) / 100.0; // cm → m
        const double std_xy = RMSE * M_SQRT1_2;
        const double noise_x = std_xy * normal(gen);
        const double noise_y = std_xy * normal(gen);

        c.x += noise_x;
        c.y += noise_y;

        // dystansu nie aktualizujemy (spójność z pipeline’em SLAM)
        visible.cones.emplace_back(std::move(c));
    }

    return visible;
}

  Track shoot_a_frame_lidar(const Track& global_track, const ParamBank& P, const State& state)
{
    // ============================================================
    // 1. Prefetch parametrów lidaru
    // ============================================================
    const double max_range_m            = P.get("lidar_max_range_m");
    const double azimuth_window_deg     = P.get("lidar_azimuth_window_deg");
    const double azimuth_window_rad     = azimuth_window_deg * M_PI / 180.0;
    const double half_azimuth_window    = 0.5 * azimuth_window_rad;

    const double sigma_r                = P.get("lidar_range_noise_sigma_m");
    const double sigma_az_base_rad      = P.get("lidar_azimuth_noise_base_rad");
    const double sigma_az_quant_rad     = P.get("lidar_azimuth_quantization_sigma_rad");
    const double roi_period_s           = P.get("lidar_roi_period_s");

    const double x_lidar_to_cog         = P.get("x_lidar_to_cog");
    const double y_lidar_to_cog         = P.get("y_lidar_to_cog");
    const double z_lidar_to_cog         = P.get("z_lidar_to_cog");
    const double lidar_pitch_rad        = P.get("lidar_pitch_rad");

    const bool use_motion_distortion    = (P.get("lidar_use_motion_distortion") > 0.5);

    // Jeśli u Ciebie pole nazywa się inaczej, zmień tylko tę linię:
    const double yaw_rate               = state.yaw_rate;

    // ============================================================
    // 2. Prefetch trygonometrii
    // ============================================================
    const double cos_yaw   = std::cos(state.yaw);
    const double sin_yaw   = std::sin(state.yaw);

    const double cos_pitch = std::cos(lidar_pitch_rad);
    const double sin_pitch = std::sin(lidar_pitch_rad);

    // ============================================================
    // 3. RNG
    // ============================================================
    static thread_local std::mt19937 gen{std::random_device{}()};
    std::normal_distribution<double> normal01(0.0, 1.0);

    // ============================================================
    // 4. Wyjście
    // ============================================================
    Track visible;
    visible.cones.reserve(global_track.cones.size());

    // ============================================================
    // 5. Główna pętla
    // ============================================================
    for (const auto& cone_global : global_track.cones)
    {
        // --------------------------------------------------------
        // 5.1. Global -> vehicle/body frame (bez pitchu lidaru)
        // --------------------------------------------------------
        const double dx = cone_global.x - state.x;
        const double dy = cone_global.y - state.y;

        const double x_body =  dx * cos_yaw + dy * sin_yaw;
        const double y_body = -dx * sin_yaw + dy * cos_yaw;
        const double z_body =  cone_global.z;

        // --------------------------------------------------------
        // 5.2. Przesunięcie do początku ramki lidaru
        //      (zgodnie z konwencją używaną już w kamerze)
        // --------------------------------------------------------
        const double x_rel = x_body + x_lidar_to_cog;
        const double y_rel = y_body + y_lidar_to_cog;
        const double z_rel = z_body + z_lidar_to_cog;

        // --------------------------------------------------------
        // 5.3. Obrót do właściwej ramki lidaru (pitch)
        //      parent->child = R_y(pitch)
        // --------------------------------------------------------
        Track_cone c;
        c.x =  cos_pitch * x_rel + sin_pitch * z_rel;
        c.y =  y_rel;
        c.z =  -sin_pitch * x_rel + cos_pitch * z_rel;
        c.color = cone_global.color;

        // --------------------------------------------------------
        // 5.4. Geometria 2D pomiaru lidaru
        // --------------------------------------------------------
        const double r_true  = std::sqrt(c.x * c.x + c.y * c.y);
        const double az_true = std::atan2(c.y, c.x);

        // Odrzuć za lidarem
        if (c.x <= 0.0) continue;

        // Odrzuć spoza zasięgu
        if (r_true > max_range_m) continue;

        // Odrzuć spoza ROI w azymucie
        if (std::abs(az_true) > half_azimuth_window) continue;

        // --------------------------------------------------------
        // 5.5. Motion distortion (brak deskew)
        //      sigma_t = T_roi / sqrt(12)
        //      sigma_az_motion = |yaw_rate| * sigma_t
        // --------------------------------------------------------
        double sigma_az_motion_rad = 0.0;
        if (use_motion_distortion)
        {
            sigma_az_motion_rad = std::abs(yaw_rate) * roi_period_s / std::sqrt(12.0);
        }

        const double sigma_az_total_rad = std::sqrt(
            sigma_az_base_rad  * sigma_az_base_rad +  sigma_az_motion_rad * sigma_az_motion_rad

            //sigma_az_quant_rad * sigma_az_quant_rad +
           // sigma_az_motion_rad * sigma_az_motion_rad
        );

        // --------------------------------------------------------
        // 5.6. Losowanie szumu
        // --------------------------------------------------------
        const double r_obs  = r_true  + sigma_r * normal01(gen);
        const double az_obs = az_true + sigma_az_total_rad * normal01(gen);

        // Dodatkowe zabezpieczenie, żeby nie wyszedł ujemny range
        if (r_obs <= 0.0) continue;

        // --------------------------------------------------------
        // 5.7. Powrót do x,y w ramce lidaru
        //      z zostawiamy z geometrii bez szumu (model 2D)
        // --------------------------------------------------------
        c.x = r_obs * std::cos(az_obs);
        c.y = r_obs * std::sin(az_obs);

        // distance ustawiamy już po zaszumieniu
        c.distance = std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z);

        visible.cones.emplace_back(std::move(c));
    }

    return visible;
}
} // namespace lem_dynamics_sim_
