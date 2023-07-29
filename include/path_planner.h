#ifndef __PATH_PLANNER_H__
#define __PATH_PLANNER_H__

#include <vector>
#include <map>
#include <algorithm>
#include <memory>
#include <stdint.h>
#include "config.h"

using position_t = std::pair<uint16_t, uint16_t>;

struct GridNode
{
    position_t position;
    float cost;      // cost from start to this node
    float heuristic; // heuristic cost from this node to end
    GridNode *parent;

    bool operator<(const GridNode &other) const
    {
        return cost + heuristic > other.cost + other.heuristic;
    }
};

class PathPlanner
{
public:
    // The map is a const reference as we do not want to modify it in our algorithms.
    // The start and end points are passed by value as they are small and easily copied.
    virtual std::vector<position_t> plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const = 0;

    void set_debug_active(bool debug_active);

protected:
    virtual float heuristic(position_t start, position_t end) const = 0;

    bool is_walkable(position_t position, const std::vector<std::vector<bool>> &map) const;
    std::vector<GridNode> get_neighbors(GridNode &current, std::map<position_t, GridNode> &all_nodes, const std::vector<std::vector<bool>> &map, position_t end = {0.0f, 0.0f}) const;
    std::vector<position_t> build_path(GridNode &endNode) const;

    void print_debug(const char *format, ...) const;

protected:
    bool m_debug_active;
    static constexpr uint16_t MAX_EXPLORED_NODES = 10000;
};

std::unique_ptr<PathPlanner> build_path_planner(PlannerType planner_type);

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Path Planning Implementations
//
///////////////////////////////////////////////////////////////////////////////////////////////////////
class AStarPathPlanner : public PathPlanner
{
public:
    std::vector<position_t> plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const override;

private:
    float heuristic(position_t start, position_t end) const override;
};

class DijkstraPathPlanner : public PathPlanner
{
public:
    std::vector<position_t> plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const override;

private:
    // No heuristic used in Dijkstra
    float heuristic(position_t start, position_t end) const override
    {
        return 0;
    };
};

class BFSPathPlanner : public PathPlanner
{
public:
    std::vector<position_t> plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const override;

private:
    // No heuristic used in BFS
    float heuristic(position_t start, position_t end) const override
    {
        return 0;
    };
};

#endif // __PATH_PLANNER_H__