#include "sim_loop.hpp"
#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <filesystem>
#include <atomic>

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using namespace lem_dynamics_sim_;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lem_simulation_node");
    ros::NodeHandle nh("~");

    // ====== 1) Pobranie parametrów z launch'a ======
    std::string param_file, track_file, log_file;
    bool logging_enabled_param;

    if (!nh.getParam("param_file", param_file))
        throw std::runtime_error("Missing required param: param_file");

    if (!nh.getParam("track_file", track_file))
        throw std::runtime_error("Missing required param: track_file");

    if (!nh.getParam("log_file", log_file))
        throw std::runtime_error("Missing required param: log_file");

    if (!nh.getParam("logging_enabled", logging_enabled_param))
        throw std::runtime_error("Missing required param: logging_enabled");

    ROS_INFO_STREAM("LEM Simulation node starting...");
    ROS_INFO_STREAM("  → param_file = " << param_file);
    ROS_INFO_STREAM("  → track_file = " << track_file);
    ROS_INFO_STREAM("  → log_file   = " << log_file);

    // ====== 2) Inicjalizacja symulatora ======
    Simulation_lem_ros_node sim(nh, param_file, track_file,
                                logging_enabled_param ? log_file : "");

    // ====== 3) Async spinner dla callbacków ROS-a ======
    // Ten spinner pracuje w osobnym wątku, żeby nie blokować pętli symulacji
    ros::AsyncSpinner ros_spinner(2); // 2 wątki -> subskrypcje i serwisy
    ros_spinner.start();

    // ====== 4) Flaga kontrolna zatrzymania ======
    std::atomic<bool> running{true};

    // ====== 5) Wątek symulacji ======
    std::thread sim_thread([&]() {
        const auto step_target = std::chrono::microseconds(1000); // 1kHz
        auto t_last = std::chrono::steady_clock::now();

        while (ros::ok() && running.load()) {
            auto t_start = std::chrono::steady_clock::now();

            try {
                sim.step();  // jeden krok symulacji
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("[SIM_LOOP] Exception: " << e.what());
            }

            auto t_end = std::chrono::steady_clock::now();
            auto step_dur = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

            // dokładne taktowanie
            if (step_dur < step_target)
                std::this_thread::sleep_for(step_target - step_dur);
            else if (step_dur > 5 * step_target)
                ROS_WARN_THROTTLE(2.0, "Simulation running slow (%.3f ms)", step_dur.count() / 1000.0);
        }
    });

    // ====== 6) Główna pętla czeka na koniec ======
    ros::waitForShutdown();
    running.store(false);

    if (sim_thread.joinable())
        sim_thread.join();

    ROS_INFO("LEM Simulation node stopped gracefully.");
    return 0;
}
