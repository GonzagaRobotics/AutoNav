#pragma once

#include <utility>
#include <cmath>
#include <memory>
#include "auto_nav_interfaces/Types.hpp"

/**
 * A site represents an area of land that we want to find a path through.
 * It contains obstacle data and a bounding box of latitudes and longitudes.
 */
class Site
{
private:
    int width;
    int height;
    double latNorth;
    double lngEast;
    double latSouth;
    double lngWest;

    /**
     * 1D array of width * height elements.
     * Each byte is packed, with the first 2 bits being obstacle north/south and east/west.
     */
    unsigned char *data;

public:
    /**
     * Creates a new site.
     *
     * @param width The width of the site.
     * @param height The height of the site.
     *
     * @param latNorth The northern latitude of the site in decimal degrees.
     * @param lngEast The eastern longitude of the site in decimal degrees.
     * @param latSouth The southern latitude of the site in decimal degrees.
     * @param lngWest The western longitude of the site in decimal degrees.
     * @param data The obstacle data for the site. It must be width * height bytes long and packed correctly.
     */
    Site(int width, int height, double latNorth, double lngEast, double latSouth, double lngWest, unsigned char *data);
    ~Site();

    int getWidth() const;
    int getHeight() const;
    double getLatNorth() const;
    double getLngEast() const;
    double getLatSouth() const;
    double getLngWest() const;

    GeoLoc getGeoLoc(int x, int y) const;
    std::pair<int, int> getXY(const GeoLoc &geoLoc) const;

    /**
     * Check if a given point is an obstacle. Any out of bounds point is an obstacle.
     *
     * @param x The x coordinate of the point.
     * @param y The y coordinate of the point.
     * @param northSouth Whether to check if the point is an obstacle in the north/south direction.
     * @param eastWest Whether to check if the point is an obstacle in the east/west direction.
     *
     * @return True if the point is an obstacle, false otherwise.
     */
    bool isObstacle(int x, int y, bool northSouth, bool eastWest) const;

    /**
     * Checks if a ray from start to end collides with any obstacles.
     *
     * @param start The start point of the ray.
     * @param end The end point of the ray.
     *
     * @return True if the ray collides with an obstacle, false otherwise.
     *
     * @see https://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
     */
    bool rayCast(std::pair<int, int> start, std::pair<int, int> end) const;
};