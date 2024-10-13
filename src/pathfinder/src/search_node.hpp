#pragma once

#include <functional>

/**
 * A node in the search space.
 *
 * Note that this overloads the equality, less than, and hash functions.
 */
struct SearchNode
{
    int x;
    int y;
    bool northSouthObstacle;
    bool eastWestObstacle;

    /**
     * G cost. The cost to move from the starting node to this node.
     */
    int g;

    /**
     * H cost. The estimated cost to move from this node to the end node.
     */
    int h;

    /**
     * The parent node of this node. Can be nullptr.
     */
    SearchNode *parent;

    /**
     * Returns the F cost, which is the sum of the G and H costs.
     */
    int f() const
    {
        return this->g + this->h;
    }

    bool operator==(const SearchNode &other) const
    {
        return this->x == other.x && this->y == other.y;
    }

    bool operator!=(const SearchNode &other) const
    {
        return !(*this == other);
    }

    bool operator<(const SearchNode *other) const
    {
        return (this->f() < other->f()) || (this->f() == other->f() && this->h < other->h);
    }

    size_t operator()(const SearchNode *node) const
    {
        return std::hash<int>()(node->x) ^ (std::hash<int>()(node->y) << 1);
    }

    bool operator()(const SearchNode *a, const SearchNode *b) const
    {
        return a->f() > b->f() || (a->f() == b->f() && a->h > b->h);
    }
};