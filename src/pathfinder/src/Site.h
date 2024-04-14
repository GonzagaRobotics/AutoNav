#pragma once

#include <utility>
#include <cmath>
#include <memory>
#include "auto_nav_interfaces/Types.h"

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

    bool isObstacle(int x, int y, bool northSouth, bool eastWest) const;

    // https://playtechs.blogspot.com/2007/03/raytracing-on-grid.html

    bool rayCast(std::pair<int, int> start, std::pair<int, int> end) const;
};