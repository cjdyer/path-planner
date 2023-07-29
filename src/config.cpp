#include "config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>

static const std::map<std::string, PlannerType> PLANNER_TYPE_LOOKUP = {
    {"ASTAR", PlannerType::ASTAR},
    {"DIJKSTRA", PlannerType::DIJKSTRA}};

void Config::set_file_path(const std::string &file_path)
{
    YAML::Node config_yaml = YAML::LoadFile(file_path);

    window_dimensions.x = config_yaml["window_width"].as<uint16_t>();
    window_dimensions.y = config_yaml["window_height"].as<uint16_t>();

    std::vector<uint16_t> start_point_conf = config_yaml["start_point"].as<std::vector<uint16_t>>();
    std::vector<uint16_t> end_point_conf = config_yaml["end_point"].as<std::vector<uint16_t>>();

    start_point.x = start_point_conf[0];
    start_point.y = start_point_conf[1];
    end_point.x = end_point_conf[0];
    end_point.y = end_point_conf[1];

    scale = config_yaml["scale"].as<uint16_t>();

    map = std::vector<std::vector<bool>>(window_dimensions.x / scale, std::vector<bool>(window_dimensions.y / scale, false)); // initialize map with false

    std::string planner_type_string = config_yaml["planner_type"].as<std::string>();

    if (PLANNER_TYPE_LOOKUP.find(planner_type_string) == PLANNER_TYPE_LOOKUP.end())
    {
        std::cout << "Invalid planner_type: " << planner_type_string << " please choose from: ASTRA or DIJKSTRA" << std::endl;
        throw std::runtime_error("Invalid planner_type in config");
    }

    planner_type = PLANNER_TYPE_LOOKUP.at(planner_type_string);

    // Pull debug state from env-vars
    char *debug_env = std::getenv("DEBUG");
    debug_active = (debug_env != nullptr);

    //! TODO: Check if config is valid
}
