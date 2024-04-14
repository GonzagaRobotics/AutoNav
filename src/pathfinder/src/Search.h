#pragma once

#include <future>
#include <atomic>
#include <cmath>
#include "auto_nav_interfaces/Types.h"
#include "deps/astar.h"
#include "Site.h"

class MapNode
{
private:
    std::shared_ptr<Site> site;

public:
    int x, y;

    MapNode() : x(0), y(0)
    {
    }

    MapNode(int px, int py, std::shared_ptr<Site> psite) : x(px), y(py), site(psite)
    {
    }

    MapNode(const GeoLoc loc, const std::shared_ptr<Site> psite) : site(psite)
    {
        auto [x, y] = site->getXY(loc);
        this->x = x;
        this->y = y;
    }

    float GoalDistanceEstimate(MapNode &nodeGoal) const;

    bool IsGoal(MapNode &nodeGoal) const;

    bool GetSuccessors(AStarSearch<MapNode> *search, MapNode *parentNode);

    float GetCost(MapNode &successor) const;

    bool IsSameState(MapNode &rhs) const;

    bool IsAdjacent(MapNode &rhs) const;

    [[nodiscard]] size_t Hash() const;
};
