#ifndef DV_2D_PP_SIM_CONFIGHELPER_H
#define DV_2D_PP_SIM_CONFIGHELPER_H
#include <string>

#include "Simulation.h"

struct Vec2;

class ConfigHelper {
    public:
    static void LoadTrackCSV(const std::string& path, Simulation& sim);
    static void SaveConesCSV(const std::string &path, const Simulation& sim);
    static void SaveTrackCSV(const std::string &path, const Simulation& sim);
};

#endif //DV_2D_PP_SIM_CONFIGHELPER_H