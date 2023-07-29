#include "path_planner.h"
#include <queue>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdarg.h>

AStarPathPlanner::AStarPathPlanner()
{
    char *debug_env = std::getenv("DEBUG");
    debug = (debug_env != nullptr);
}

void AStarPathPlanner::print_debug(const char *format, ...) const
{
    if (debug)
    {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsprintf(buffer, format, args);
        va_end(args);
        std::cout << buffer << std::endl;
    }
}

std::vector<position_t> AStarPathPlanner::plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const
{
    print_debug("Planning path from (%d,%d) to (%d,%d)", start.first, start.second, end.first, end.second);

    uint16_t nodes_explored = 0;
    std::priority_queue<GridNode> openList;
    std::map<position_t, GridNode> allNodes;

    GridNode startNode = {start, 0, heuristic(start, end), nullptr};
    allNodes[start] = startNode;
    openList.push(startNode);

    print_debug("Pushed start node into openList: position=(%d,%d), g=%.2f, h=%.2f", startNode.position.first, startNode.position.second, startNode.g, startNode.h);

    while (!openList.empty())
    {
        GridNode current = openList.top();
        openList.pop();

        print_debug("Popped node from openList: position=(%d,%d), g=%.2f, h=%.2f", current.position.first, current.position.second, current.g, current.h);

        if (current.position == end)
        {
            print_debug("Found path to end node");
            return buildPath(allNodes[current.position]);
        }

        std::vector<GridNode> neighbors = getNeighbors(allNodes[current.position], allNodes, map, end);

        for (GridNode &neighbor : neighbors)
        {
            print_debug("Checking neighbor: position=(%d,%d), g=%.2f, h=%.2f", neighbor.position.first, neighbor.position.second, neighbor.g, neighbor.h);

            auto it = allNodes.find(neighbor.position);
            if (it == allNodes.end() || neighbor.g < it->second.g)
            {
                print_debug("Updated neighbor in allNodes and pushed into openList: position=(%d,%d), g=%.2f, h=%.2f", neighbor.position.first, neighbor.position.second, neighbor.g, neighbor.h);
                allNodes[neighbor.position] = neighbor;
                openList.push(neighbor);
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

std::vector<AStarPathPlanner::GridNode> AStarPathPlanner::getNeighbors(GridNode &current, std::map<position_t, GridNode> &allNodes, const std::vector<std::vector<bool>> &map, position_t end) const
{
    std::vector<GridNode> neighbors;
    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            if (dx == 0 && dy == 0)
                continue; // skip current node

            position_t newPosition = {current.position.first + dx, current.position.second + dy};

            if (isWalkable(newPosition, map))
            {

                float newG = current.g + std::abs(dx) + std::abs(dy);
                float newH = heuristic(newPosition, end);

                print_debug("Found walkable neighbor: position=(%d,%d), g=%.2f, h=%.2f", newPosition.first, newPosition.second, newG, newH);

                GridNode newNode = {newPosition, newG, newH, &allNodes[current.position]};
                neighbors.push_back(newNode);
            }
        }
    }
    return neighbors;
}

bool AStarPathPlanner::isWalkable(position_t position, const std::vector<std::vector<bool>> &map) const
{
    // Check if position is within the map bounds
    if (position.first >= 0 && position.second >= 0 && position.first < map.size() && position.second < map[0].size())
    {
        // If the map cell is false, it's walkable
        return !map[position.first][position.second];
    }
    // Position is out of map bounds
    return false;
}

std::vector<position_t> AStarPathPlanner::buildPath(GridNode &endNode) const
{
    std::vector<position_t> path;
    GridNode *currentNode = &endNode;
    while (currentNode)
    {
        print_debug("Added node to path: position=(%d,%d)", currentNode->position.first, currentNode->position.second);

        path.push_back(currentNode->position);
        currentNode = currentNode->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}
