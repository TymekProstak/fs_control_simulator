#include "../include/ConfigHelper.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <ros/package.h>

#include "common/Vec2.h"
#include "../include/Simulation.h"

char *OpenFile() {
    char *filename = new char[512]; // pls dont overflow );
    char cwd[1024];
    char command[2048] = R"(zenity --file-selection --title="Select a config file" 2> /dev/null)";
    FILE *f = nullptr;
    if (std::string cwd = ros::package::getPath("dv_2d_pp_sim"); !cwd.empty())
        sprintf(command, R"(zenity --save --file-selection --title="Save Config" --filename="%s/" 2> /dev/null)",
                cwd.c_str());
    else if (auto homedir = getenv("HOME"); homedir != nullptr) {
        sprintf(command,
                R"(zenity --save --file-selection --title="Save Config" --filename="%s/dv_ws/src/dv_2d_pp_sim/" 2> /dev/null)",
                homedir);
    }

    f = popen(command, "r");
    auto res = fgets(filename, 512, f);
    if (!res) {
        delete[] filename;
        return nullptr;
    }
    res[strlen(res) - 1] = 0;

    pclose(f);

    return res;
}

char *SaveFile() {
    char *filename = new char[512]; // pls dont overflow );
    char command[2048] = R"(zenity --save --file-selection --title="Save Config" 2> /dev/null)";
    FILE *f = nullptr;
    if (std::string cwd = ros::package::getPath("dv_2d_pp_sim"); !cwd.empty())
        sprintf(command, R"(zenity --save --file-selection --title="Save Config" --filename="%s/" 2> /dev/null)",
                cwd.c_str());
    else if (auto homedir = getenv("HOME"); homedir != nullptr) {
        sprintf(command,
                R"(zenity --save --file-selection --title="Save Config" --filename="%s/dv_ws/src/dv_2d_pp_sim/" 2> /dev/null)",
                homedir);
    }

    f = popen(command, "r");
    auto res = fgets(filename, 512, f);
    if (!res) {
        delete[] filename;
        return nullptr;
    }
    res[strlen(res) - 1] = 0;

    pclose(f);

    return res;
}

void ConfigHelper::LoadTrackCSV(const std::string& path, Simulation& sim) {
    try {
        std::string filename = path;
        if (path.empty()) {
            if (auto user_file = OpenFile(); user_file)
                filename = std::string(user_file);
        }

        std::ifstream file(filename);
        if (!file.is_open())
            throw std::runtime_error("Could not open file!");
        std::string line;
        sim.center_points.clear();
        while (std::getline(file, line)) {
            if (line.size() < 2)
                continue;
            // replace all commas with spaces in line
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream iss(line);
            if (isalpha(line[0]) && isalpha(line[1])) {
                std::string key, value;
                iss >> key; iss >> value;
                if (key == "close_loop")
                    sim.close_loop = value == "1" || value == "true";
                else if (key == "smooth_curve")
                    sim.smooth_curve = value == "1" || value == "true";
                else if (key == "cones_distance")
                    sim.cones_min_distance = std::stof(value);
                else if (key == "cones_padding")
                    sim.cones_padding = std::stof(value);
                else if (key == "cones_proximity_remove_threshold")
                    sim.cones_max_distance = std::stof(value);
            }
            else {
                Vec2 point;
                if (!(iss >> point.x >> point.y))
                    continue;
                sim.center_points.push_back(point);
            }
        }
    }
    catch (std::exception& e) {
        std::cout << "Error while reading track points!\n" << e.what() << std::endl;
    }
}

void ConfigHelper::SaveConesCSV(const std::string& path, const Simulation& sim) {
    try {
        std::string filename = path;
        if (path.empty()) {
            if (auto user_file = SaveFile(); user_file)
                filename = std::string(user_file);
        }

        std::ofstream file(filename);
        if (!file.is_open())
            throw std::runtime_error("Could not open file!");

        for (auto& p : sim.cones_right_positions)
            file << p.x << "," << p.y << ",yellow\n";

        for (auto& p : sim.cones_left_positions)
            file << p.x << "," << p.y << ",blue\n";
    }
    catch (std::exception& e) {
        std::cout << "Error while exporting cones!\n" << e.what() << std::endl;
    }
}

void ConfigHelper::SaveTrackCSV(const std::string &path, const Simulation& sim) {
    try {
        std::string filename = path;
        if (path.empty()) {
            if (auto user_file = SaveFile(); user_file)
                filename = std::string(user_file);
        }

        std::ofstream file(filename);
        if (!file.is_open())
            throw std::runtime_error("Could not open file!");

        file << "close_loop" << "," << sim.close_loop << std::endl;
        file << "smooth_curve" << "," << sim.smooth_curve << std::endl;
        file << "cones_min_distance" << "," << sim.cones_min_distance << std::endl;
        file << "cones_padding" << "," << sim.cones_padding << std::endl;
        file << "cones_max_distance" << "," << sim.cones_max_distance << std::endl;

        for (auto& p : sim.center_points) {
            file << p.x << "," << p.y << std::endl;
        }
    }
    catch (std::exception& e) {
        std::cout << "Error while exporting track points!\n" << e.what() << std::endl;
    }
}
