#include "path_planner.h"

#include "path_planner.h"
#include <queue>
#include <iostream>
#include <memory>
#include <unordered_map>

std::vector<position_t> BFSPathPlanner::plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const
{
    print_debug("Planning path from (%d,%d) to (%d,%d)", start.first, start.second, end.first, end.second);

    uint16_t nodes_explored = 0;
    std::queue<BFSGridNode> open_list;
    std::map<position_t, BFSGridNode> all_nodes;

    BFSGridNode start_node = {start, nullptr};
    all_nodes[start] = start_node;
    open_list.push(start_node);

    print_debug("Pushed start node into open_list: position=(%d,%d)", start_node.position.first, start_node.position.second);

    while (!open_list.empty())
    {
        BFSGridNode current = open_list.front();
        open_list.pop();

        print_debug("Popped node from open_list: position=(%d,%d)", current.position.first, current.position.second);

        if (current.position == end)
        {
            print_debug("Found path to end node");
            return build_path(all_nodes[current.position]);
        }

        std::vector<BFSGridNode> neighbors = get_neighbors(current, all_nodes, map);

        for (BFSGridNode &neighbor : neighbors)
        {
            print_debug("Checking neighbor: position=(%d,%d)", neighbor.position.first, neighbor.position.second);

            auto it = all_nodes.find(neighbor.position);
            if (it == all_nodes.end())
            {
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

std::vector<BFSPathPlanner::BFSGridNode> BFSPathPlanner::get_neighbors(BFSGridNode &current, std::map<position_t, BFSGridNode> &all_nodes, const std::vector<std::vector<bool>> &map) const
{
    std::vector<BFSGridNode> neighbors;

    for (const position_t &direction : directions)
    {

        position_t new_position = {current.position.first + direction.first, current.position.second + direction.second};

        if (!is_walkable(new_position, map))
        {
            continue;
        }

        print_debug("Found walkable neighbor: position=(%d,%d)", new_position.first, new_position.second);

        BFSGridNode new_node = {new_position, &all_nodes[current.position]};
        neighbors.push_back(new_node);
    }

    return neighbors;
}

std::vector<position_t> BFSPathPlanner::build_path(BFSGridNode &end_node) const
{
    std::vector<position_t> path;
    BFSGridNode *current_node = &end_node;
    while (current_node)
    {
        print_debug("Added node to path: position=(%d,%d)", current_node->position.first, current_node->position.second);

        path.push_back(current_node->position);
        current_node = current_node->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}