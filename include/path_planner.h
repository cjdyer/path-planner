#ifndef __PATH_PLANNER_H__
#define __PATH_PLANNER_H__

#include <vector>
#include <map>
#include <algorithm>
#include <memory>
#include <future>
#include <stdint.h>
#include "config.h"

using position_t = std::pair<uint16_t, uint16_t>;

class PathPlanner
{
public:
    virtual std::future<std::vector<position_t>> plan_path_async(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const = 0;
    void set_debug_active(bool debug_active);

protected:
    struct GridNode
    {
        position_t position;
        float cost;
        float heuristic;
        GridNode *parent;

        bool operator<(const GridNode &other) const
        {
            return (cost + heuristic) < (other.cost + other.heuristic);
        }
    };

    inline bool is_walkable(position_t position, const std::vector<std::vector<bool>> &map) const
    {

        if (position.first < 0 || position.second < 0 || position.first > (map.size() - 1) || position.second > (map[0].size() - 1))
        {
            // Position is out of map bounds - return early
            return false;
        }

        // If the map cell is false, it's walkable
        return !map[position.first][position.second];
    }

    std::vector<position_t> build_path(GridNode &end_node) const;

    void print_debug(const char *format, ...) const;

protected:
    const std::vector<position_t> directions = {{1, 1}, {1, 0}, {1, -1}, {0, 1}, {0, -1}, {-1, 1}, {-1, 0}, {-1, -1}};
    static constexpr uint16_t MAX_EXPLORED_NODES = 10000;

private:
    bool m_debug_active;
};

std::unique_ptr<PathPlanner> build_path_planner(PlannerType planner_type);

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Path Planning Implementations
//
///////////////////////////////////////////////////////////////////////////////////////////////////////
class BFSPathPlanner : public PathPlanner
{
public:
    std::future<std::vector<position_t>> plan_path_async(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const override
    {
        return std::async(std::launch::async, &BFSPathPlanner::plan_path, this, map, start, end);
    }

private:
    std::vector<position_t> plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const;
    std::vector<GridNode> get_neighbors(GridNode &current, std::map<position_t, GridNode> &all_nodes, const std::vector<std::vector<bool>> &map) const;
};

class DijkstraPathPlanner : public PathPlanner
{
public:
    std::future<std::vector<position_t>> plan_path_async(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const override
    {
        return std::async(std::launch::async, &DijkstraPathPlanner::plan_path, this, map, start, end);
    }

private:
    std::vector<position_t> plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const;
    std::vector<GridNode> get_neighbors(GridNode &current, std::map<position_t, GridNode> &all_nodes, const std::vector<std::vector<bool>> &map) const;
};

class AStarPathPlanner : public PathPlanner
{
public:
    std::future<std::vector<position_t>> plan_path_async(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const override
    {
        return std::async(std::launch::async, &AStarPathPlanner::plan_path, this, map, start, end);
    }

private:
    std::vector<position_t> plan_path(const std::vector<std::vector<bool>> &map, position_t start, position_t end) const;
    std::vector<GridNode> get_neighbors(GridNode &current, std::map<position_t, GridNode> &all_nodes, const std::vector<std::vector<bool>> &map, position_t end) const;
    float heuristic(position_t start, position_t end) const;
};

#endif // __PATH_PLANNER_H__