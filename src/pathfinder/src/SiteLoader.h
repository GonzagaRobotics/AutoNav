#pragma once

#include <string>
#include <memory>
#include <stdexcept>
#include <fstream>
#include "Site.h"
#include "deps/stb_image.h"

class SiteLoader
{
private:
    std::string name;

    void loadBounds(double &latNorth, double &lngEast, double &latSouth, double &lngWest);

public:
    SiteLoader() = delete;

    SiteLoader(std::string name) : name(name)
    {
    }

    std::shared_ptr<Site> load();
};