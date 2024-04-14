#pragma once

#include <string>
#include <fstream>
#include <stdexcept>
#include <iomanip>
#include "auto_nav_interfaces/Types.h"

void debugKML(const std::string &name, const GeoLoc &start, const Plan &plan);