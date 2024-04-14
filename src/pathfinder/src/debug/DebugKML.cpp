#include "DebugKML.h"

void debugKML(const std::string &name, const GeoLoc &start, const Plan &plan)
{
    auto infile = std::ifstream("/home/damon/robotics/AutoNav/static/kml_debug_template.txt");

    if (!infile.is_open())
    {
        throw std::runtime_error("Could not open kml_debug_template.txt");
    }

    auto outfile = std::ofstream(name);

    if (!outfile.is_open())
    {
        infile.close();
        throw std::runtime_error("Could not open " + name);
    }

    std::string line;

    while (std::getline(infile, line))
    {
        if (line.find("%WAYPOINTS%") != std::string::npos)
        {
            // C++ generally outputs 6 significant digits, which is not enough
            auto prevPrecision = outfile.precision(10);

            // Write the start location to the output file (because it's not in the plan)
            outfile << start.longitude << "," << start.latitude << ",0\n";

            // Write the waypoints to the output file
            for (const auto &waypoint : plan.waypoints)
            {
                outfile << waypoint.longitude << "," << waypoint.latitude << ",0\n";
            }

            outfile.precision(prevPrecision);
        }
        else
        {
            if (line.find("%NAME%") != std::string::npos)
            {
                line.replace(line.find("%NAME%"), 6, name);
            }

            outfile << line << "\n";
        }
    }

    infile.close();
    outfile.close();
}