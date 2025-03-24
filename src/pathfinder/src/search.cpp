#include "search.hpp"

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

std::vector<const SearchNode *> Search::retracePath(const SearchNode *start, const SearchNode *end) const
{
    std::vector<const SearchNode *> path;

    const SearchNode *current = end;

    while (current != start)
    {
        path.push_back(current);
        current = current->parent;
    }

    // Since we are retracing the path from the end to the start, we need to reverse it.
    std::reverse(path.begin(), path.end());

    return path;
}

std::pair<bool, int> Search::canReach(SearchNode *start, SearchNode *end, std::atomic<bool> &pathfinding)
{
    // We're using a BFS approach to check if we can reach the end node

    // The start or end being an obstacle is kind of a non-starter
    if (start->northSouthObstacle || start->eastWestObstacle || end->northSouthObstacle || end->eastWestObstacle)
    {
        return std::make_pair(false, getDistance(start, end));
    }

    std::queue<SearchNode *> set;
    bool canReach = false;
    int distance = 0;

    set.push(start);
    distance = getDistance(start, end);

    // To keep track of nodes the algorithm has already visited, we'll set the
    // g-cost to 1 and use it as a flag.
    start->g = 1;

    while (!set.empty() && pathfinding)
    {
        SearchNode *current = set.front();
        set.pop();

        if (*current == *end)
        {
            canReach = true;
            break;
        }

        current->g = 1;

        std::vector<SearchNode *> neighbors;
        this->getNeighbors(neighbors, current);

        for (SearchNode *neighbor : neighbors)
        {
            if (neighbor->g == 0)
            {
                set.push(neighbor);
                neighbor->g = 1;

                int newDistance = getDistance(neighbor, end);

                if (newDistance < distance)
                {
                    distance = newDistance;
                }
            }
        }
    }

    // Reset the g-costs
    for (SearchNode &node : this->allNodes)
    {
        node.g = 0;
    }

    return std::make_pair(canReach, distance);
}

SearchNode *Search::expandGridSearch(int startX, int startY, int minSteps)
{
    // We're going to follow the pattern of starting facing south,
    // 1 step, turn, 1 step, turn, 2 steps, turn, 2 steps, turn, 3 steps, etc.
    // We start at 1 step per turn, do it twice, then increase the steps per turn by 1.

    int totalSteps = 0;

    int currentX = startX;
    int currentY = startY;
    bool currentValid = false;
    int stepsPerTurn = 1;
    int steps = 0;
    // North: 0, East: 1, South: 2, West: 3
    int dir = 2;

    while (totalSteps < minSteps || !currentValid)
    {
        // Move in the current direction
        switch (dir)
        {
        case 0:
            currentY--;
            break;
        case 1:
            currentX++;
            break;
        case 2:
            currentY++;
            break;
        case 3:
            currentX--;
            break;
        }

        // Check if the new position is valid
        auto currentNode = getNode(currentX, currentY);
        currentValid = currentNode != nullptr && !(currentNode->northSouthObstacle || currentNode->eastWestObstacle);

        steps++;
        totalSteps++;

        // Turn if we've reached the number of steps for this direction
        if (steps == stepsPerTurn)
        {
            dir = (dir + 1) % 4;
            steps = 0;

            // Increase the steps per turn every other turn
            if (dir == 0 || dir == 2)
            {
                stepsPerTurn++;
            }
        }
    }

    return getNode(currentX, currentY);
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

std::vector<const SearchNode *> Search::search(SearchNode *start, SearchNode *end, std::atomic<bool> &pathfinding)
{
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
    return std::vector<const SearchNode *>();
}

std::vector<GeoLoc> Search::simplifyPath(const std::vector<const SearchNode *> &path) const
{
    std::vector<GeoLoc> simplified;

    const SearchNode *target = path.back();
    const SearchNode *current = path.front();

    while (current != target)
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
        target = path.back();
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

    // In case we modify the start or end node, we'll keep the original nodes
    auto origStartNode = startNode;
    auto origEndNode = endNode;

    // There are a couple cases where the algorithm can't find a path on it's own:
    // 1. The start or end node is out of bounds
    // 2. The start or end node is an obstacle
    // 3. The end node is unreachable from the start node

    // In these cases, we still need to do our best to return a path.
    // If the start or end node is out of bounds or an obstacle, we'll use an
    // expanding grid search to find the closest node we can reach.
    // Start by seeing if we can reach the end node. If we can't, we'll use the
    // expanding grid search to find another node to use as the start or end node,
    // depending on which one is more problematic by some kind of heuristic.
    // From there, we'll see if the new pair is reachable, if not, we'll repeat
    // the expanding grid search with a higher minimum number of steps. Once we
    // have a pair of nodes that are reachable, we can perform the search. Then
    // we can add the unreachable points to the path and hope for the best.

    // First check if the start or end node is out of bounds, if so, find one in bounds
    if (startNode == nullptr)
    {
        startNode = expandGridSearch(site->getXY(start).first, site->getXY(start).second, 1);
    }

    if (endNode == nullptr)
    {
        endNode = expandGridSearch(site->getXY(end).first, site->getXY(end).second, 1);
    }

    auto canReachResult = canReach(startNode, endNode, pathfinding);
    int gridMinSteps = 1;
    bool lastWasEnd = false;

    while (canReachResult.first == false)
    {
        // We need to determine whether the start or end node is more problematic.
        // We know the closest distance from the end node to the fill, so we will
        // change the start node if the closest fill distance is greater than half
        // the distance from the start to the end node.

        int referenceDistance = getDistance(startNode, endNode) / 2;

        if (canReachResult.second < referenceDistance)
        {
            if (lastWasEnd == false)
            {
                gridMinSteps = 1;
            }

            endNode = expandGridSearch(origEndNode->x, origEndNode->y, gridMinSteps);
            gridMinSteps *= 2;
            lastWasEnd = true;
        }
        else
        {
            if (lastWasEnd)
            {
                gridMinSteps = 1;
            }

            startNode = expandGridSearch(origStartNode->x, origStartNode->y, gridMinSteps);
            gridMinSteps *= 2;
            lastWasEnd = false;
        }

        canReachResult = canReach(startNode, endNode, pathfinding);
    }

    // Perform the search

    auto path = search(startNode, endNode, pathfinding);

    if (path.size() == 0)
    {
        return std::make_pair(std::vector<GeoLoc>(), "This should never happen");
    }

    auto simplePath = simplifyPath(path);

    if (endNode != origEndNode)
    {
        if (origEndNode)
        {
            simplePath.push_back(site->getGeoLoc(origEndNode->x, origEndNode->y));
        }
        else
        {
            simplePath.push_back(end);
        }
    }

    return std::make_pair(simplePath, "");
}
