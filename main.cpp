#include "sim_loop.hpp"
#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <filesystem>

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using namespace lem_dynamics_sim_;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lem_simulation_node");
    ros::NodeHandle nh("~"); // prywatny namespace, np. /lem_sim

    // ====== 1) Pobranie parametrów z launcha lub ustawienie domyślnych ======
    std::string param_file, track_file, log_file;

    nh.param<std::string>("param_file", param_file,
        (fs::current_path() / "config" / "params.json").string());
    nh.param<std::string>("track_file", track_file,
        (fs::current_path() / "tracks" / "track.json").string());
    nh.param<std::string>("log_file", log_file,
        (fs::current_path() / "logs" / "sim_log.csv").string());

    ROS_INFO_STREAM("LEM Simulation node starting...");
    ROS_INFO_STREAM("  → param_file = " << param_file);
    ROS_INFO_STREAM("  → track_file = " << track_file);
    ROS_INFO_STREAM("  → log_file   = " << log_file);

    bool logging_enabled_param;
    nh.param<bool>("logging_enabled", logging_enabled_param, false); // logowanie domyślnie wyłączone ->
    // gdy na false to konstruktor dostanie pusty string jako log_file -> nie będzie logował

    // ====== 2) Inicjalizacja symulatora ======
    Simulation_lem_ros_node sim(nh, param_file, track_file,
        logging_enabled_param ? log_file : "");

    // ====== 3) Pętla główna – taktowanie 1 kHz ======
    const double dt_target = 0.001; // 1 ms
    const auto   step_target = std::chrono::microseconds(1000);

    ros::Rate backup_rate(500); // tylko awaryjny (jeśli coś się desynchronizuje)

    auto t_last = std::chrono::steady_clock::now();

    while (ros::ok())
    {
        // --- a) Aktualny czas startu kroku
        auto t_start = std::chrono::steady_clock::now();

        // --- b) Wykonaj krok symulacji
        sim.step();

        // --- c) Oblicz ile trwał krok
        auto t_end = std::chrono::steady_clock::now();
        auto step_dur = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

        // --- d) Uśpij, aby zachować dokładnie 1 ms
        if (step_dur < step_target)
            std::this_thread::sleep_for(step_target - step_dur);
        else if (step_dur > 5 * step_target)
            ROS_WARN_THROTTLE(2.0, "Simulation running slow (%.3f ms)", step_dur.count() / 1000.0);

        // --- e) Obsłuż ROS callbacki
        ros::spinOnce();
    }

    ROS_INFO("LEM Simulation node stopped gracefully.");
    return 0;
}
