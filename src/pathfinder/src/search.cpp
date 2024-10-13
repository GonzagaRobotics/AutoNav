#include "search.hpp"

SearchNode *Search::getNode(int x, int y)
{
    if (x < 0 || x >= this->site->getWidth() || y < 0 || y >= this->site->getHeight())
    {
        return nullptr;
    }

    return &(this->allNodes[y * this->site->getWidth() + x]);
}

void Search::getNeighbors(std::vector<SearchNode *> &neighbors, const SearchNode *node)
{
    // Since we are reusing the neighbors vector, we need to clear it first.
    neighbors.clear();

    auto north = getNode(node->x, node->y - 1);
    auto east = getNode(node->x + 1, node->y);
    auto south = getNode(node->x, node->y + 1);
    auto west = getNode(node->x - 1, node->y);

    // North
    if (north != nullptr && !north->northSouthObstacle)
    {
        neighbors.push_back(getNode(node->x, node->y - 1));
    }

    // North-East
    if (
        north != nullptr && east != nullptr &&
        ((!north->eastWestObstacle && !east->northSouthObstacle) ||
         (!north->northSouthObstacle && !east->eastWestObstacle)))
    {
        neighbors.push_back(getNode(node->x + 1, node->y - 1));
    }

    // East
    if (east != nullptr && !east->eastWestObstacle)
    {
        neighbors.push_back(getNode(node->x + 1, node->y));
    }

    // South-East
    if (
        south != nullptr && east != nullptr &&
        ((!south->eastWestObstacle && !east->northSouthObstacle) ||
         (!south->northSouthObstacle && !east->eastWestObstacle)))
    {
        neighbors.push_back(getNode(node->x + 1, node->y + 1));
    }

    // South
    if (south != nullptr && !south->northSouthObstacle)
    {
        neighbors.push_back(getNode(node->x, node->y + 1));
    }

    // South-West
    if (
        south != nullptr && west != nullptr &&
        ((!south->eastWestObstacle && !west->northSouthObstacle) ||
         (!south->northSouthObstacle && !west->eastWestObstacle)))
    {
        neighbors.push_back(getNode(node->x - 1, node->y + 1));
    }

    // West
    if (west != nullptr && !west->eastWestObstacle)
    {
        neighbors.push_back(getNode(node->x - 1, node->y));
    }

    // North-West
    if (
        north != nullptr && west != nullptr &&
        ((!north->eastWestObstacle && !west->northSouthObstacle) ||
         (!north->northSouthObstacle && !west->eastWestObstacle)))
    {
        neighbors.push_back(getNode(node->x - 1, node->y - 1));
    }
}

int Search::getDistance(const SearchNode *nodeA, const SearchNode *nodeB) const
{
    // We are using a variation of the Octile distance heuristic,
    // where we use 10 and 14 instead of 1 and sqrt(2) respectively.

    int xDist = std::abs(nodeA->x - nodeB->x);
    int yDist = std::abs(nodeA->y - nodeB->y);

    return 10 * (xDist + yDist) + (14 - 2 * 10) * std::min(xDist, yDist);
}

std::vector<SearchNode> Search::retracePath(const SearchNode *start, const SearchNode *end) const
{
    std::vector<SearchNode> path;

    const SearchNode *current = end;

    while (*current != *start)
    {
        path.push_back(*current);
        current = current->parent;
    }

    // Since we are retracing the path from the end to the start, we need to reverse it.
    std::reverse(path.begin(), path.end());

    return path;
}

Search::Search(std::shared_ptr<Site> site)
{
    this->site = site;
    this->allNodes = std::vector<SearchNode>(site->getWidth() * site->getHeight());

    // Pre-populate the allNodes vector with all the nodes in the search space
    // so we don't have to create them on the fly

    for (int x = 0; x < site->getWidth(); x++)
    {
        for (int y = 0; y < site->getHeight(); y++)
        {
            this->allNodes[y * site->getWidth() + x] =
                SearchNode{
                    x,
                    y,
                    site->isObstacle(x, y, true, false),
                    site->isObstacle(x, y, false, true),
                    0,
                    0,
                    nullptr};
        }
    }
}

std::vector<SearchNode> Search::search(SearchNode *start, SearchNode *end, std::atomic<bool> &pathfinding)
{
    if (start == nullptr || end == nullptr)
    {
        throw std::runtime_error("Start or end node is out of bounds");
    }
    else if (start->northSouthObstacle || start->eastWestObstacle || end->northSouthObstacle || end->eastWestObstacle)
    {
        throw std::runtime_error("Start or end node is an obstacle");
    }

    // Set up our open and closed sets

    std::priority_queue<SearchNode *, std::vector<SearchNode *>, SearchNode> openSet;
    // Searching through a priority queue is slow, so we have a separate set to check if a node is in the open set.
    std::unordered_set<SearchNode *, SearchNode> openSetContains;

    std::unordered_set<SearchNode *, SearchNode> closedSet;
    std::vector<SearchNode *> neighbors;

    // Start by adding the start node to the open set
    openSet.push(start);
    openSetContains.insert(start);

    // Continue searching until we have no more nodes in the open set
    while (openSet.size() > 0 && pathfinding)
    {
        // Find the node in openSet with the lowest f, or the lowest h if there is a tie
        SearchNode *current = openSet.top();

        // Remove the node from the open set and add it to the closed set
        openSet.pop();
        openSetContains.erase(current);
        closedSet.insert(current);

        // If we have reached the end node, retrace the path and return it
        if (*current == *end)
        {
            return retracePath(start, end);
        }

        // Get the neighbors of the current node and iterate through them to expand the search
        // getNeighbors will make sure any neighbors are valid and are not obstacles
        this->getNeighbors(neighbors, current);

        for (SearchNode *neighbor : neighbors)
        {
            // Skip the neighbor if it is in the closed set
            if (closedSet.find(neighbor) != closedSet.end())
            {
                continue;
            }

            // Calculate the new G cost for the neighbor and see if it's in the open set
            int newMovementCost = current->g + getDistance(current, neighbor);
            bool setContainsNeighbor = openSetContains.find(neighbor) != openSetContains.end();

            // Update the neighbor if needed
            if (newMovementCost < neighbor->g || !setContainsNeighbor)
            {
                // Update the neighbor's costs and parent
                neighbor->g = newMovementCost;
                neighbor->h = getDistance(neighbor, end);
                neighbor->parent = current;

                // Add the neighbor to the open set if it's not already there
                if (!setContainsNeighbor)
                {
                    openSet.push(neighbor);
                    openSetContains.insert(neighbor);
                }
            }
        }
    }

    // If we reach this point, there is no path from the start to the end
    return std::vector<SearchNode>();
}

std::vector<GeoLoc> Search::simplifyPath(const std::vector<SearchNode> &path) const
{
    std::vector<GeoLoc> simplified;

    const SearchNode *target = &path.back();
    const SearchNode *current = &path.front();

    while (*current != *target)
    {
        // Try and draw a straight line from the current node to the end node.
        // If we can, we can skip the nodes in between. If we can't, look at
        // the parent node and try again until a line can be drawn or we reach
        // the node adjacent to the current node.

        while (target->parent != current)
        {
            if (!site->rayCast(std::make_pair(current->x, current->y), std::make_pair(target->x, target->y)))
            {
                break;
            }

            target = target->parent;
        }

        simplified.push_back(site->getGeoLoc(target->x, target->y));

        // Reset the current and target nodes
        current = target;
        target = &path.back();
    }

    return simplified;
}

std::pair<std::vector<GeoLoc>, std::string> Search::findPath(
    GeoLoc start,
    GeoLoc end,
    std::atomic<bool> &pathfinding)
{
    // Get the start and end nodes
    auto startNode = getNode(site->getXY(start).first, site->getXY(start).second);
    auto endNode = getNode(site->getXY(end).first, site->getXY(end).second);

    // TODO: Better handling of start/end nodes that are obstacles or technically out of reach

    // Perform the search
    try
    {
        auto path = search(startNode, endNode, pathfinding);

        if (path.size() == 0)
        {
            return std::make_pair(std::vector<GeoLoc>(), "End is unreachable");
        }

        return std::make_pair(simplifyPath(path), "");
    }
    catch (const std::exception &e)
    {
        return std::make_pair(std::vector<GeoLoc>(), e.what());
    }
}
