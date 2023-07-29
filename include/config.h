#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>
#include <vector>
#include "types.h"

struct Config
{
    void set_file_path(const std::string &file_path);

    Vec2 window_dimensions;

    Vec2 start_point;
    Vec2 end_point;

    std::vector<std::vector<bool>> map;
};

static Config config;

#endif //__CONFIG_H__
