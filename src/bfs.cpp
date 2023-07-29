#include "path_planner.h"

#include "path_planner.h"
#include <queue>
#include <iostream>
#include <memory>
#include <unordered_map>

std::vector<position_t> BFSPathPlanner::plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const
{
    print_debug("Planning path from (%d,%d) to (%d,%d)", start.first, start.second, end.first, end.second);

    std::queue<GridNode> open_list;
    std::map<position_t, GridNode> all_nodes;

    GridNode start_node = {start, 0, 0, nullptr};
    all_nodes[start] = start_node;
    open_list.push(start_node);

    print_debug("Pushed start node into open_list: position=(%d,%d), cost=%.2f", start_node.position.first, start_node.position.second, start_node.cost);

    while (!open_list.empty())
    {
        GridNode current = open_list.front();
        open_list.pop();

        print_debug("Popped node from open_list: position=(%d,%d), cost=%.2f", current.position.first, current.position.second, current.cost);

        if (current.position == end)
        {
            print_debug("Found path to end node");
            return build_path(all_nodes[current.position]);
        }

        std::vector<GridNode> neighbors = get_neighbors(current, all_nodes, map);

        for (GridNode &neighbor : neighbors)
        {
            auto it = all_nodes.find(neighbor.position);
            if (it == all_nodes.end())
            {
                all_nodes[neighbor.position] = neighbor;
                open_list.push(neighbor);
            }
        }
    }

    return std::vector<position_t>(); // no path found
}