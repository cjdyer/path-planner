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
    }
}

void PathPlanner::set_debug_active(bool debug_active)
{
    m_debug_active = debug_active;
}

void PathPlanner::print_debug(const char *format, ...) const
{
    if (m_debug_active)
    {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsprintf(buffer, format, args);
        va_end(args);
        std::cout << buffer << std::endl;
    }
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

std::vector<GridNode> PathPlanner::get_neighbors(GridNode &current, std::map<position_t, GridNode> &all_nodes, const std::vector<std::vector<bool>> &map, position_t end) const
{
    std::vector<GridNode>
        neighbors;
    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            if (dx == 0 && dy == 0)
                continue; // skip current node

            position_t new_position = {current.position.first + dx, current.position.second + dy};

            if (is_walkable(new_position, map))
            {

                float new_cost = current.cost + std::sqrt(dx * dx + dy * dy);
                float new_heuristic = heuristic(new_position, end);

                print_debug("Found walkable neighbor: position=(%d,%d), cost=%.2f, heuristic=%.2f", new_position.first, new_position.second, new_cost, new_heuristic);

                GridNode new_node = {new_position, new_cost, new_heuristic, &all_nodes[current.position]};
                neighbors.push_back(new_node);
            }
        }
    }
    return neighbors;
}