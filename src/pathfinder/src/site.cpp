#include "site.hpp"

Site::Site(int width, int height, double latNorth, double lngEast, double latSouth, double lngWest, unsigned char *data)
{
    this->width = width;
    this->height = height;
    this->latNorth = latNorth;
    this->lngEast = lngEast;
    this->latSouth = latSouth;
    this->lngWest = lngWest;
    this->data = data;
}

Site::~Site()
{
    // Since the data is collected from stbi_load, it was allocated with malloc.
    // It's a good idea to use free() to deallocate it instead of delete[].
    free(data);
}

int Site::getWidth() const
{
    return this->width;
}

int Site::getHeight() const
{
    return this->height;
}

double Site::getLatNorth() const
{
    return this->latNorth;
}

double Site::getLngEast() const
{
    return this->lngEast;
}

double Site::getLatSouth() const
{
    return this->latSouth;
}

double Site::getLngWest() const
{
    return this->lngWest;
}

GeoLoc Site::getGeoLoc(int x, int y) const
{
    double lat = getLatNorth() - (getLatNorth() - getLatSouth()) * (y / (double)getHeight());
    double lng = getLngWest() + (getLngEast() - getLngWest()) * (x / (double)getWidth());

    return GeoLoc{lat, lng};
}

std::pair<int, int> Site::getXY(const GeoLoc &geoLoc) const
{
    int x = std::round(((geoLoc.longitude - getLngWest()) / (getLngEast() - getLngWest()) * getWidth()));
    int y = std::round(((getLatNorth() - geoLoc.latitude) / (getLatNorth() - getLatSouth()) * getHeight()));

    return std::make_pair(x, y);
}

bool Site::isObstacle(int x, int y, bool northSouth, bool eastWest) const
{
    // Any out of bounds is an obstacle
    if (x < 0 || x >= width || y < 0 || y >= height)
    {
        return true;
    }

    unsigned char value = data[y * width + x];

    return (northSouth && (value & 1) != 0) || (eastWest && (value & 2) != 0);
}

bool Site::rayCast(std::pair<int, int> start, std::pair<int, int> end) const
{
    int dx = abs(end.first - start.first);
    int dy = abs(end.second - start.second);
    int lx = start.first;
    int ly = start.second;
    int n = 1 + dx + dy;
    int x_inc = (end.first > start.first) ? 1 : -1;
    int y_inc = (end.second > start.second) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n)
    {
        // Since we can move through obstacles in certain conditions, we use
        // our knowledge of the direction of the ray to determine whether
        // we should check for obstacles in the north/south and/or east/west direction.
        if (isObstacle(lx, ly, dy >= dx, dx >= dy))
        {
            return true;
        }

        if (error > 0)
        {
            lx += x_inc;
            error -= dy;
        }
        else if (error < 0)
        {
            ly += y_inc;
            error += dx;
        }
        else if (error == 0)
        {
            // Ensure that perfectly diagonal lines don't take up more tiles than necessary.
            // http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html?showComment=1281448902099#c3785285092830049685
            lx += x_inc;
            ly += y_inc;
            error -= dy;
            error += dx;
            --n;
        }
    }

    return false;
}