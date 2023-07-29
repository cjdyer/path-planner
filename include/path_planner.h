#ifndef __PATH_PLANNER_H__
#define __PATH_PLANNER_H__

#include <vector>
#include <map>
#include <algorithm>
#include <stdint.h>

using position_t = std::pair<uint16_t, uint16_t>;

class PathPlanner
{
public:
    // The map is a const reference as we do not want to modify it in our algorithms.
    // The start and end points are passed by value as they are small and easily copied.
    virtual std::vector<position_t> plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const = 0;
};

class AStarPathPlanner : public PathPlanner
{
public:
    AStarPathPlanner();

    std::vector<position_t> plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const override;

private:
    struct GridNode
    {
        position_t position;
        float g; // cost from start to this node
        float h; // heuristic cost from this node to end
        GridNode *parent;

        bool operator<(const GridNode &other) const
        {
            return g + h > other.g + other.h;
        }
    };

    float heuristic(position_t start, position_t end) const;
    std::vector<GridNode> getNeighbors(GridNode &current, std::map<position_t, GridNode> &allNodes, const std::vector<std::vector<bool>> &map, position_t end) const;
    bool isWalkable(position_t position, const std::vector<std::vector<bool>> &map) const;
    std::vector<position_t> buildPath(GridNode &endNode) const;

    static constexpr uint16_t MAX_EXPLORED_NODES = 10000;

    bool debug;
    void print_debug(const char *format, ...) const;
};

#endif // __PATH_PLANNER_H__