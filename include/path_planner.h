#ifndef __PATH_PLANNER_H__
#define __PATH_PLANNER_H__

#include <vector>
#include <map>
#include <algorithm>
#include <memory>
#include <stdint.h>
#include "config.h"

using position_t = std::pair<uint16_t, uint16_t>;

class PathPlanner
{
public:
    // The map is a const reference as we do not want to modify it in our algorithms.
    // The start and end points are passed by value as they are small and easily copied.
    virtual std::vector<position_t> plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const = 0;

    void set_debug_active(bool debug_active);

protected:
    bool is_walkable(position_t position, const std::vector<std::vector<bool>> &map) const;

    void print_debug(const char *format, ...) const;

protected:
    const std::vector<position_t> directions = {{1, 1}, {1, 0}, {1, -1}, {0, 1}, {0, -1}, {-1, 1}, {-1, 0}, {-1, -1}};
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
    struct AStarGridNode;

    std::vector<AStarGridNode>
    get_neighbors(AStarGridNode &current, std::map<position_t, AStarGridNode> &all_nodes, const std::vector<std::vector<bool>> &map, position_t end) const;
    float heuristic(position_t start, position_t end) const;
    std::vector<position_t> build_path(AStarGridNode &end_node) const;

    struct AStarGridNode
    {
        position_t position;
        float cost;      // cost from start to this node
        float heuristic; // heuristic cost from this node to end
        AStarGridNode *parent;

        bool operator<(const AStarGridNode &other) const
        {
            return cost + heuristic < other.cost + other.heuristic;
        }
    };
};

class DijkstraPathPlanner : public PathPlanner
{
public:
    std::vector<position_t> plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const override;

private:
    struct DijkstraGridNode;

    std::vector<DijkstraGridNode> get_neighbors(DijkstraGridNode &current, std::map<position_t, DijkstraGridNode> &all_nodes, const std::vector<std::vector<bool>> &map) const;
    std::vector<position_t> build_path(DijkstraGridNode &end_node) const;

    struct DijkstraGridNode
    {
        position_t position;
        float cost; // cost from start to this node
        DijkstraGridNode *parent;

        bool operator<(const DijkstraGridNode &other) const
        {
            return cost < other.cost;
        }
    };
};

class BFSPathPlanner : public PathPlanner
{
public:
    std::vector<position_t> plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const override;

private:
    struct BFSGridNode;

    std::vector<BFSGridNode> get_neighbors(BFSGridNode &current, std::map<position_t, BFSGridNode> &all_nodes, const std::vector<std::vector<bool>> &map) const;
    std::vector<position_t> build_path(BFSGridNode &end_node) const;

    struct BFSGridNode
    {
        position_t position;
        BFSGridNode *parent;
    };
};

#endif // __PATH_PLANNER_H__