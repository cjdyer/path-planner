#include "path_planner.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <stdarg.h>

std::unique_ptr<PathPlanner> build_path_planner(PlannerType planner_type)
{
    switch (planner_type)
    {
    case PlannerType::ASTAR:
        return std::make_unique<AStarPathPlanner>();
    case PlannerType::DIJKSTRA:
        return std::make_unique<DijkstraPathPlanner>();
    case PlannerType::BFS:
        return std::make_unique<BFSPathPlanner>();
    }

    return nullptr;
}

void PathPlanner::set_debug_active(bool debug_active)
{
    m_debug_active = debug_active;
}

void PathPlanner::print_debug(const char *format, ...) const
{
    if (!m_debug_active)
    {
        return;
    }

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);
    std::cout << buffer << std::endl;
}

bool PathPlanner::is_walkable(position_t position, const std::vector<std::vector<bool>> &map) const
{
    if (position.first < 0 || position.second < 0 || position.first > (map.size() - 1) || position.second > (map[0].size() - 1))
    {
        // Position is out of map bounds - return early
        return false;
    }

    // If the map cell is false, it's walkable
    return !map[position.first][position.second];
}