#include "Search.h"

float MapNode::GoalDistanceEstimate(MapNode &nodeGoal) const
{
    int xDist = std::abs(x - nodeGoal.x);
    int yDist = std::abs(y - nodeGoal.y);

    return (float)std::sqrt(xDist * xDist + yDist * yDist);
}

bool MapNode::IsGoal(MapNode &nodeGoal) const
{
    return x == nodeGoal.x && y == nodeGoal.y;
}

bool MapNode::GetSuccessors(AStarSearch<MapNode> *search, MapNode *parentNode)
{
    using obstacle = std::pair<bool, bool>;

    int parentX = -1;
    int parentY = -1;

    if (parentNode)
    {
        parentX = parentNode->x;
        parentY = parentNode->y;
    }

    MapNode newNode;
    obstacle obstacleWest, obstacleEast, obstacleNorth, obstacleSouth;

    obstacleNorth.first = site->isObstacle(x, y - 1, true, false);
    obstacleNorth.second = site->isObstacle(x, y - 1, false, true);

    obstacleEast.first = site->isObstacle(x + 1, y, true, false);
    obstacleEast.second = site->isObstacle(x + 1, y, false, true);

    obstacleSouth.first = site->isObstacle(x, y + 1, true, false);
    obstacleSouth.second = site->isObstacle(x, y + 1, false, true);

    obstacleWest.first = site->isObstacle(x - 1, y, true, false);
    obstacleWest.second = site->isObstacle(x - 1, y, false, true);

    // North
    if (!obstacleNorth.first && !((parentX == x) && (parentY == y - 1)))
    {
        newNode = MapNode(x, y - 1, site);
        search->AddSuccessor(newNode);
    }

    // North-East
    if (!obstacleNorth.first && !obstacleEast.second && !((parentX == x + 1) && (parentY == y - 1)))
    {
        newNode = MapNode(x + 1, y - 1, site);
        search->AddSuccessor(newNode);
    }

    // East
    if (!obstacleEast.second && !((parentX == x + 1) && (parentY == y)))
    {
        newNode = MapNode(x + 1, y, site);
        search->AddSuccessor(newNode);
    }

    // South-East
    if (!obstacleSouth.first && !obstacleEast.second && !((parentX == x + 1) && (parentY == y + 1)))
    {
        newNode = MapNode(x + 1, y + 1, site);
        search->AddSuccessor(newNode);
    }

    // South
    if (!obstacleSouth.first && !((parentX == x) && (parentY == y + 1)))
    {
        newNode = MapNode(x, y + 1, site);
        search->AddSuccessor(newNode);
    }

    // South-West
    if (!obstacleSouth.first && !obstacleWest.second && !((parentX == x - 1) && (parentY == y + 1)))
    {
        newNode = MapNode(x - 1, y + 1, site);
        search->AddSuccessor(newNode);
    }

    // West
    if (!obstacleWest.second && !((parentX == x - 1) && (parentY == y)))
    {
        newNode = MapNode(x - 1, y, site);
        search->AddSuccessor(newNode);
    }

    // North-West
    if (!obstacleNorth.first && !obstacleWest.second && !((parentX == x - 1) && (parentY == y - 1)))
    {
        newNode = MapNode(x - 1, y - 1, site);
        search->AddSuccessor(newNode);
    }

    return true;
}

float MapNode::GetCost(MapNode &successor) const
{
    return 1.0f;
}

bool MapNode::IsSameState(MapNode &rhs) const
{
    return x == rhs.x && y == rhs.y;
}

bool MapNode::IsAdjacent(MapNode &rhs) const
{
    return std::abs(x - rhs.x) <= 1 && std::abs(y - rhs.y) <= 1;
}

size_t MapNode::Hash() const
{
    size_t h1 = std::hash<float>{}((float)x);
    size_t h2 = std::hash<float>{}((float)y);

    return h1 ^ (h2 << 1);
}
