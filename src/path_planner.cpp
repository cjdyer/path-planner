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

std::vector<position_t> PathPlanner::build_path(GridNode &end_node) const
{
    std::vector<position_t> path;
    GridNode *current_node = &end_node;
    while (current_node)
    {
        print_debug("Added node to path: position=(%d,%d)", current_node->position.first, current_node->position.second);

        path.push_back(current_node->position);
        current_node = current_node->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}