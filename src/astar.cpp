#include "path_planner.h"
#include <stdarg.h>
#include <queue>
#include <cmath>

std::vector<position_t> AStarPathPlanner::plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const
{
    print_debug("Planning path from (%d,%d) to (%d,%d)", start.first, start.second, end.first, end.second);

    uint16_t nodes_explored = 0;
    std::queue<GridNode> open_list;
    std::map<position_t, GridNode> all_nodes;

    GridNode startNode = {start, 0, heuristic(start, end), nullptr};
    all_nodes[start] = startNode;
    open_list.push(startNode);

    print_debug("Pushed start node into open_list: position=(%d,%d), cost=%.2f, heuristic=%.2f", startNode.position.first, startNode.position.second, startNode.cost, startNode.heuristic);

    while (!open_list.empty())
    {
        GridNode current = open_list.front();
        open_list.pop();

        print_debug("Popped node from open_list: position=(%d,%d), cost=%.2f, heuristic=%.2f", current.position.first, current.position.second, current.cost, current.heuristic);

        if (current.position == end)
        {
            print_debug("Found path to end node");
            return build_path(all_nodes[current.position]);
        }

        std::vector<GridNode> neighbors = get_neighbors(all_nodes[current.position], all_nodes, map, end);

        for (GridNode &neighbor : neighbors)
        {
            print_debug("Checking neighbor: position=(%d,%d), cost=%.2f, heuristic=%.2f", neighbor.position.first, neighbor.position.second, neighbor.cost, neighbor.heuristic);

            auto it = all_nodes.find(neighbor.position);
            if (it == all_nodes.end() || neighbor.cost < it->second.cost)
            {
                print_debug("Updated neighbor in all_nodes and pushed into open_list: position=(%d,%d), cost=%.2f, heuristic=%.2f", neighbor.position.first, neighbor.position.second, neighbor.cost, neighbor.heuristic);
                all_nodes[neighbor.position] = neighbor;
                open_list.push(neighbor);
            }
        }

        if (++nodes_explored > MAX_EXPLORED_NODES)
        {
            print_debug("Exceeded maximum number of nodes to explore.");
            return std::vector<position_t>(); // no path found
        }
    }

    print_debug("No path found to end node");
    return std::vector<position_t>(); // no path found
}

float AStarPathPlanner::heuristic(position_t start, position_t end) const
{
    int dx = abs(start.first - end.first);
    int dy = abs(start.second - end.second);
    int min = std::min(dx, dy);
    int max = std::max(dx, dy);
    return (max - min) + std::sqrt(2) * min;
}