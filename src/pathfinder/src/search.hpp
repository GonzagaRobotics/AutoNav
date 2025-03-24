#pragma once

#include <memory>
#include <cmath>
#include <utility>
#include <vector>
#include <unordered_set>
#include <queue>
#include <algorithm>
#include <stdexcept>
#include <atomic>
#include "auto_nav_interfaces/Types.hpp"
#include "site.hpp"
#include "search_node.hpp"

/**
 * Any-angle A* search algorithm.
 */
class Search
{
private:
    /**
     * The site to search on.
     */
    std::shared_ptr<Site> site;

    /**
     * All nodes in the search space.
     */
    std::vector<SearchNode> allNodes;

    /**
     * Gets a pointer to the node at the given coordinates. Will return nullptr if the coordinates are out of bounds.
     */
    SearchNode *getNode(int x, int y)
    {
        if (x < 0 || x >= this->site->getWidth() || y < 0 || y >= this->site->getHeight())
        {
            return nullptr;
        }

        return &(this->allNodes[y * this->site->getWidth() + x]);
    }

    /**
     * Calculates and returns the distance between two nodes.
     */
    int getDistance(const SearchNode *nodeA, const SearchNode *nodeB) const
    {
        // We are using a variation of the Octile distance heuristic,
        // where we use 10 and 14 instead of 1 and sqrt(2) respectively.

        int xDist = std::abs(nodeA->x - nodeB->x);
        int yDist = std::abs(nodeA->y - nodeB->y);

        return 10 * (xDist + yDist) + (14 - 2 * 10) * std::min(xDist, yDist);
    }

    /**
     * Gets the neighbors of the given node and stores them in the neighbors vector.
     *
     * The function will clear the neighbors vector before adding any neighbors.
     *
     * @param neighbors The vector to store the neighbors in.
     * @param node The node to get the neighbors of.
     */
    void getNeighbors(std::vector<SearchNode *> &neighbors, const SearchNode *node);

    /**
     * Retraces the path from the end node to the start node.
     *
     * @param start The start node.
     * @param end The end node.
     *
     * @return The path from the start node to the end node.
     */
    std::vector<const SearchNode *> retracePath(const SearchNode *start, const SearchNode *end) const;

    /**
     * Determines if the end node can be reached from the start node.
     *
     * @return A pair where the first element is true if the end node can be reached from the start node and false otherwise.
     * The second element is the closest distance from the fill to the end node as an octile distance.
     */
    std::pair<bool, int> canReach(SearchNode *start, SearchNode *end, std::atomic<bool> &pathfinding);

    /**
     * Performs an expanding grid search from the start node to a node that is
     * both not an obstacle and is minSteps away from the start node.
     */
    SearchNode *expandGridSearch(int startX, int startY, int minSteps);

    /**
     * Performs the search.
     *
     * @param start The start node.
     * @param end The end node.
     *
     * @return The path from the start node to the end node. If the path is empty, the end is unreachable.
     */
    std::vector<const SearchNode *> search(SearchNode *start, SearchNode *end, std::atomic<bool> &pathfinding);

    /**
     * Simplifies the path by performing any-angle path smoothing.
     *
     * @param path The path to simplify.
     *
     * @return The simplified path.
     */
    std::vector<GeoLoc> simplifyPath(const std::vector<const SearchNode *> &path) const;

public:
    Search(std::shared_ptr<Site> site);

    std::pair<std::vector<GeoLoc>, std::string> findPath(
        GeoLoc start,
        GeoLoc end,
        std::atomic<bool> &pathfinding);
};