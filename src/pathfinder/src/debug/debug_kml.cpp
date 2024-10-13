#include "debug_kml.hpp"

void debugKML(const std::string &static_dir, const GeoLoc &start, const Plan &plan)
{
    auto infile = std::ifstream(static_dir + "kml_debug_template.txt");

    if (!infile.is_open())
    {
        throw std::runtime_error("Could not open kml_debug_template.txt");
    }

    std::string outName =
        "path_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count());

    auto outfile = std::ofstream(static_dir + outName + ".kml");

    if (!outfile.is_open())
    {
        infile.close();
        throw std::runtime_error("Could not open output file");
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

            // Reset the precision so we don't mess up other output
            outfile.precision(prevPrecision);
        }
        else
        {
            if (line.find("%NAME%") != std::string::npos)
            {
                line.replace(line.find("%NAME%"), 6, outName);
            }

            outfile << line << "\n";
        }
    }

    infile.close();
    outfile.close();
}