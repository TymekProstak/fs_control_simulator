#include <fstream>
#include <stdexcept>

#include <ros/ros.h>
#include <nlohmann/json.hpp>

#include "complementary/ParamBank.hpp"
#include "complementary/wrapper.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "complementary_kinematic_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        std::string config_json_path;
        if (!pnh.getParam("config_json_path", config_json_path))
        {
            throw std::runtime_error("Missing required ROS param '~config_json_path'");
        }

        std::ifstream file(config_json_path);
        if (!file.is_open())
        {
            throw std::runtime_error("Could not open config JSON: " + config_json_path);
        }

        nlohmann::json J;
        file >> J;

        complementary_filter_kinematic::ParamBank param_bank =
            complementary_filter_kinematic::build_param_bank(J);

        complementary_filter_kinematic::ComplementaryFilterWrapper wrapper(nh, pnh, param_bank);
        wrapper.spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("[complementary_kinematic_node] fatal error: " << e.what());
        return 1;
    }

    return 0;
}