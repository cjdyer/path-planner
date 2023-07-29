#include "config.h"
#include <yaml-cpp/yaml.h>

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

    map = std::vector<std::vector<bool>>(window_dimensions.y, std::vector<bool>(window_dimensions.x, false)); // initialize map with false

    for (const auto &segment : config_yaml["wall_segments"])
    {
        std::vector<uint16_t> segment_coords = segment.as<std::vector<uint16_t>>();
        uint16_t startX = segment_coords[0];
        uint16_t startY = segment_coords[1];
        uint16_t width = segment_coords[2];
        uint16_t height = segment_coords[3];

        for (uint16_t i = 0; i < width; i++)
        {
            for (uint16_t j = 0; j < height; j++)
            {
                map[startY + j][startX + i] = true; // mark cell as wall
            }
        }
    }

    //! TODO: Check if config is valid
}
