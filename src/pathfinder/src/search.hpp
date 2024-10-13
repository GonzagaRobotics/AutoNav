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
    SearchNode *getNode(int x, int y);

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
     * Calculates and returns the distance between two nodes.
     */
    int getDistance(const SearchNode *nodeA, const SearchNode *nodeB) const;

    /**
     * Retraces the path from the end node to the start node.
     *
     * @param start The start node.
     * @param end The end node.
     *
     * @return The path from the start node to the end node.
     */
    std::vector<SearchNode> retracePath(const SearchNode *start, const SearchNode *end) const;

    /**
     * Performs the search.
     *
     * @param start The start node.
     * @param end The end node.
     *
     * @return The path from the start node to the end node. If the path is empty, the end is unreachable.
     *
     * @throws std::runtime_error If the start or end node is out of bounds or an obstacle.
     */
    std::vector<SearchNode> search(SearchNode *start, SearchNode *end, std::atomic<bool> &pathfinding);

    /**
     * Simplifies the path by performing any-angle path smoothing.
     *
     * @param path The path to simplify.
     *
     * @return The simplified path.
     */
    std::vector<GeoLoc> simplifyPath(const std::vector<SearchNode> &path) const;

public:
    Search(std::shared_ptr<Site> site);

    std::pair<std::vector<GeoLoc>, std::string> findPath(
        GeoLoc start,
        GeoLoc end,
        std::atomic<bool> &pathfinding);
};