#ifndef __TYPES_H__
#define __TYPES_H__

#include <map>

enum class PlannerType
{
    ASTAR,
    DIJKSTRA,
    BFS
};

static const std::map<std::string, PlannerType> PLANNER_TYPE_LOOKUP = {
    {"ASTAR", PlannerType::ASTAR},
    {"DIJKSTRA", PlannerType::DIJKSTRA},
    {"BFS", PlannerType::BFS}};

enum class MouseDraggingType
{
    NONE,
    START,
    END,
    MAP_ADD,
    MAP_REMOVE
};

struct Vec2
{
    float x, y;

    bool operator<(const Vec2 &rhs) const
    {
        if (x < rhs.x)
            return true;
        if (rhs.x < x)
            return false;
        return y < rhs.y;
    }

    bool operator==(const Vec2 &other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Vec2 &other) const
    {
        return !(*this == other);
    }
};

#endif // __TYPES_H__