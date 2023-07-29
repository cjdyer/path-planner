#include "path_planner.h"
#include <stdarg.h>
#include <queue>
#include <cmath>

std::vector<position_t> AStarPathPlanner::plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const
{
    print_debug("Planning path from (%d,%d) to (%d,%d)", start.first, start.second, end.first, end.second);

    uint16_t nodes_explored = 0;
    std::queue<AStarGridNode> open_list;
    std::map<position_t, AStarGridNode> all_nodes;

    AStarGridNode start_node = {start, 0.0f, heuristic(start, end), nullptr};
    all_nodes[start] = start_node;
    open_list.push(start_node);

    print_debug("Pushed start node into open_list: position=(%d,%d), cost=%.2f, heuristic=%.2f", start_node.position.first, start_node.position.second, start_node.cost, start_node.heuristic);

    while (!open_list.empty())
    {
        AStarGridNode current = open_list.front();
        open_list.pop();

        print_debug("Popped node from open_list: position=(%d,%d), cost=%.2f, heuristic=%.2f", current.position.first, current.position.second, current.cost, current.heuristic);

        if (current.position == end)
        {
            print_debug("Found path to end node");
            return build_path(all_nodes[current.position]);
        }

        std::vector<AStarGridNode> neighbors = get_neighbors(current, all_nodes, map, end);

        for (AStarGridNode &neighbor : neighbors)
        {
            print_debug("Checking neighbor: position=(%d,%d), cost=%.2f, heuristic=%.2f", neighbor.position.first, neighbor.position.second, neighbor.cost, neighbor.heuristic);

            auto it = all_nodes.find(neighbor.position);
            if (it == all_nodes.end() || neighbor < it->second)
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

std::vector<AStarPathPlanner::AStarGridNode> AStarPathPlanner::get_neighbors(AStarGridNode &current, std::map<position_t, AStarGridNode> &all_nodes, const std::vector<std::vector<bool>> &map, position_t end) const
{
    std::vector<AStarGridNode> neighbors;

    for (const position_t &direction : directions)
    {
        position_t new_position = {current.position.first + direction.first, current.position.second + direction.second};

        if (!is_walkable(new_position, map))
        {
            continue;
        }

        float new_cost = current.cost + 1;

        if (direction.first != 0 && direction.second != 0)
        {
            // Both are 1 or -1 so add sqrt(2)
            new_cost += 0.414213562373;
        }

        float new_heuristic = heuristic(new_position, end);

        print_debug("Found walkable neighbor: position=(%d,%d) cost=%.2f heuristic=%.2f", new_position.first, new_position.second, new_cost, new_heuristic);

        AStarGridNode new_node = {new_position, new_cost, new_heuristic, &all_nodes[current.position]};
        neighbors.push_back(new_node);
    }

    return neighbors;
}

float AStarPathPlanner::heuristic(position_t start, position_t end) const
{
    int dx = abs(start.first - end.first);
    int dy = abs(start.second - end.second);
    int min = std::min(dx, dy);
    int max = std::max(dx, dy);
    return (max - min) + std::sqrt(2) * min;
}

std::vector<position_t> AStarPathPlanner::build_path(AStarGridNode &end_node) const
{
    std::vector<position_t> path;
    AStarGridNode *current_node = &end_node;
    while (current_node)
    {
        print_debug("Added node to path: position=(%d,%d)", current_node->position.first, current_node->position.second);

        path.push_back(current_node->position);
        current_node = current_node->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}