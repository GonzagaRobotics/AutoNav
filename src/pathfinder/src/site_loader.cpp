#include "site_loader.hpp"

void SiteLoader::loadBounds(
    const std::string &name,
    double &latNorth,
    double &lngEast,
    double &latSouth,
    double &lngWest)
{
    auto file = std::ifstream(pathToDir + name + "_bounds.txt");

    if (!file.is_open())
    {
        throw std::runtime_error("Bounds file not found for " + name);
    }

    std::string line;

    while (std::getline(file, line))
    {
        // Split the line into key and value
        auto pos = line.find(' ');

        if (pos == std::string::npos)
        {
            file.close();
            throw std::runtime_error("Invalid bounds file format");
        }

        auto key = line.substr(0, pos);
        auto value = line.substr(pos + 1);

        if (key == "final_lat_north")
        {
            latNorth = std::stod(value);
        }
        else if (key == "final_lng_east")
        {
            lngEast = std::stod(value);
        }
        else if (key == "final_lat_south")
        {
            latSouth = std::stod(value);
        }
        else if (key == "final_lng_west")
        {
            lngWest = std::stod(value);
        }
    }

    file.close();
}

SiteLoader::SiteLoader(const std::string &pathToDir)
{
    this->pathToDir = pathToDir;
}

std::shared_ptr<Site> SiteLoader::load(const std::string &name)
{
    double latNorth, lngEast, latSouth, lngWest;
    loadBounds(name, latNorth, lngEast, latSouth, lngWest);

    int width, height, channels;
    auto filename = pathToDir + name + "_slope.png";

    unsigned char *imageData = stbi_load(filename.c_str(), &width, &height, &channels, 0);

    if (imageData == nullptr)
    {
        throw std::runtime_error("Failed to load image: " + filename);
    }

    if (channels != 1)
    {
        stbi_image_free(imageData);
        throw std::runtime_error("Expected 1 channel in " + filename + ", got " + std::to_string(channels));
    }

    return std::make_shared<Site>(width, height, latNorth, lngEast, latSouth, lngWest, imageData);
}