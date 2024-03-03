#define STB_IMAGE_IMPLEMENTATION

#include <vector>
#include <utility>
#include <cmath>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "auto_nav_msgs/msg/geo_point.hpp"
#include "auto_nav_msgs/msg/plan.hpp"
#include "stb_image.h"
#include "astar.h"

using namespace rclcpp;
using GeoPointMsg = auto_nav_msgs::msg::GeoPoint;
using PlanMsg = auto_nav_msgs::msg::Plan;

const double leftLon = -110.822933;
const double rightLon = -110.760651;
const double topLat = 38.430790;
const double bottomLat = 38.381782;

const int width = 5431;
const int height = 5444;
bool map[width * height] = {};

const char *filename = "/app/ros/static/slope_binary_filled.tga";
const char *kmlTemplate = R"(
    <?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
<name>Paths</name>
<description>Examples of paths. Note that the tessellate tag is by default
  set to 0. If you want to create tessellated lines, they must be authored
  (or edited) directly in KML.</description>
<Style id="yellowLineGreenPoly">
  <LineStyle>
    <color>7f00ffff</color>
    <width>4</width>
  </LineStyle>
  <PolyStyle>
    <color>7f00ff00</color>
  </PolyStyle>
</Style>
<Placemark>
  <name>Absolute Extruded</name>
  <description>Transparent green wall with yellow outlines</description>
  <styleUrl>#yellowLineGreenPoly</styleUrl>
  <LineString>
    <extrude>1</extrude>
    <tessellate>1</tessellate>
    <gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode>
    <coordinates>
        %WAYPOINTS%
    </coordinates>
  <LineString>
</Placemark>
</Document>
</kml>)";

typedef std::pair<double, double> Waypoint;

bool IsObstacle(int x, int y)
{
    if (x < 0 || x >= width || y < 0 || y >= height)
    {
        return true;
    }

    return map[y * width + x];
}

unsigned char *loadImage()
{
    int tW, tH, tC;
    unsigned char *data = stbi_load(filename, &tW, &tH, &tC, 0);

    if (data == nullptr)
    {
        throw std::runtime_error("Failed to load image");
    }

    return data;
}

class MapNode
{
public:
    int x, y;

    MapNode() : x(0), y(0)
    {
    }

    MapNode(int x, int y) : x(x), y(y)
    {
    }

    float GoalDistanceEstimate(MapNode &nodeGoal) const;

    bool IsGoal(MapNode &nodeGoal) const;

    bool GetSuccessors(AStarSearch<MapNode> *search, MapNode *parentNode);

    float GetCost(MapNode &successor) const;

    bool IsSameState(MapNode &rhs) const;

    [[nodiscard]] size_t Hash() const;
};

float MapNode::GoalDistanceEstimate(MapNode &nodeGoal) const
{
    int xDist = std::abs(x - nodeGoal.x);
    int yDist = std::abs(y - nodeGoal.y);

    return (float)std::sqrt(xDist * xDist + yDist * yDist);
    //    return (float) (xDist + yDist);
}

bool MapNode::IsGoal(MapNode &nodeGoal) const
{
    return x == nodeGoal.x && y == nodeGoal.y;
}

bool MapNode::GetSuccessors(AStarSearch<MapNode> *search, MapNode *parentNode)
{
    int parentX = -1;
    int parentY = -1;

    if (parentNode)
    {
        parentX = parentNode->x;
        parentY = parentNode->y;
    }

    MapNode newNode;
    bool obstacleLeft = IsObstacle(x - 1, y);
    bool obstacleRight = IsObstacle(x + 1, y);
    bool obstacleUp = IsObstacle(x, y - 1);
    bool obstacleDown = IsObstacle(x, y + 1);

    // Left
    if (!obstacleLeft && !((parentX == x - 1) && (parentY == y)))
    {
        newNode = MapNode(x - 1, y);
        search->AddSuccessor(newNode);
    }

    // Right
    if (!obstacleRight && !((parentX == x + 1) && (parentY == y)))
    {
        newNode = MapNode(x + 1, y);
        search->AddSuccessor(newNode);
    }

    // Up
    if (!obstacleUp && !((parentX == x) && (parentY == y - 1)))
    {
        newNode = MapNode(x, y - 1);
        search->AddSuccessor(newNode);
    }

    // Down
    if (!obstacleDown && !((parentX == x) && (parentY == y + 1)))
    {
        newNode = MapNode(x, y + 1);
        search->AddSuccessor(newNode);
    }

    // // Left Up
    if (!IsObstacle(x - 1, y - 1) && !((parentX == x - 1) && (parentY == y - 1)))
    {
        // But not if there is an obstacle to the left and up
        if (!obstacleRight && !obstacleUp)
        {
            newNode = MapNode(x - 1, y - 1);
            search->AddSuccessor(newNode);
        }
    }

    // // Right Up
    if (!IsObstacle(x + 1, y - 1) && !((parentX == x + 1) && (parentY == y - 1)))
    {
        // But not if there is an obstacle to the right and up
        if (!obstacleLeft && !obstacleUp)
        {
            newNode = MapNode(x + 1, y - 1);
            search->AddSuccessor(newNode);
        }
    }

    // // Left Down
    if (!IsObstacle(x - 1, y + 1) && !((parentX == x - 1) && (parentY == y + 1)))
    {
        // But not if there is an obstacle to the left and down
        if (!obstacleLeft && !obstacleDown)
        {
            newNode = MapNode(x - 1, y + 1);
            search->AddSuccessor(newNode);
        }
    }

    // // Right Down
    if (!IsObstacle(x + 1, y + 1) && !((parentX == x + 1) && (parentY == y + 1)))
    {
        // But not if there is an obstacle to the right and down
        if (!obstacleRight && !obstacleDown)
        {
            newNode = MapNode(x + 1, y + 1);
            search->AddSuccessor(newNode);
        }
    }

    return true;
}

float MapNode::GetCost(MapNode &successor) const
{
    return 1.0f;
}

bool MapNode::IsSameState(MapNode &rhs) const
{
    return x == rhs.x && y == rhs.y;
}

size_t MapNode::Hash() const
{
    size_t h1 = std::hash<float>{}((float)x);
    size_t h2 = std::hash<float>{}((float)y);

    return h1 ^ (h2 << 1);
}

Waypoint nodeToLatLon(MapNode node)
{
    double lon = leftLon + (rightLon - leftLon) * (node.x / (double)width);
    double lat = topLat - (topLat - bottomLat) * (node.y / (double)height);

    return Waypoint{lat, lon};
}

MapNode latLonToNode(Waypoint waypoint)
{
    int x = (int)((waypoint.second - leftLon) / (rightLon - leftLon) * width);
    int y = (int)((topLat - waypoint.first) / (topLat - bottomLat) * height);

    return MapNode{x, y};
}

// https://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
bool raytrace(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int x = x0;
    int y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n)
    {
        if (IsObstacle(x, y))
        {
            return true;
        }

        if (error > 0)
        {
            x += x_inc;
            error -= dy;
        }
        else if (error < 0)
        {
            y += y_inc;
            error += dx;
        }
        else if (error == 0)
        {
            // Ensure that perfectly diagonal lines don't take up more tiles than necessary.
            // http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html?showComment=1281448902099#c3785285092830049685
            x += x_inc;
            y += y_inc;
            error -= dy;
            error += dx;
            --n;
        }
    }

    return false;
}

class PathFinder : public Node
{
private:
    GeoPointMsg currentLocation;

    Publisher<PlanMsg>::SharedPtr planPublisher;
    Subscription<GeoPointMsg>::SharedPtr targetSubscriber;

    std::vector<Waypoint> pathfind(Waypoint start, Waypoint end)
    {
        MapNode startNode = latLonToNode(start);
        MapNode endNode = latLonToNode(end);

        RCLCPP_INFO(get_logger(), "Start: %d, %d", startNode.x, startNode.y);
        RCLCPP_INFO(get_logger(), "End: %d, %d", endNode.x, endNode.y);

        if (IsObstacle(startNode.x, startNode.y) || IsObstacle(endNode.x, endNode.y))
        {
            throw std::runtime_error("Start or end is an obstacle");
        }

        AStarSearch<MapNode> search(width * height);

        search.SetStartAndGoalStates(startNode, endNode);

        unsigned int searchState;
        unsigned int searchSteps = 0;

        do
        {
            searchSteps++;
            searchState = search.SearchStep();
        } while (searchState == AStarSearch<MapNode>::SEARCH_STATE_SEARCHING);

        auto waypoints = std::vector<Waypoint>{};

        RCLCPP_INFO(get_logger(), "Search steps: %d", searchSteps);

        // Lazy path simplification
        if (searchState == AStarSearch<MapNode>::SEARCH_STATE_SUCCEEDED)
        {
            RCLCPP_INFO(get_logger(), "Simplifying path...");

            MapNode *node = search.GetSolutionStart();
            MapNode *segmentStart = node;

            node = search.GetSolutionNext();
            MapNode *segmentEnd = node;

            std::string waypointsString;

            while (node)
            {
                if (raytrace(segmentStart->x, segmentStart->y, segmentEnd->x, segmentEnd->y))
                {
                    // RCLCPP_INFO(get_logger(), "Raytrace hit between (%d, %d) and (%d, %d)", segmentStart->x, segmentStart->y, segmentEnd->x, segmentEnd->y);
                    auto latLon = nodeToLatLon(*segmentStart);

                    waypointsString += std::to_string(latLon.second) + "," + std::to_string(latLon.first) + ",0\n";

                    waypoints.push_back(latLon);
                    segmentStart = segmentEnd;
                }

                node = search.GetSolutionNext();
                segmentEnd = node;
            }

            auto latLon = nodeToLatLon(*segmentStart);
            waypointsString += std::to_string(latLon.second) + "," + std::to_string(latLon.first) + ",0";
            waypoints.push_back(latLon);

            std::string kml = kmlTemplate;
            kml.replace(kml.find("%WAYPOINTS%"),
                        std::string("%WAYPOINTS%").size(),
                        waypointsString);

            RCLCPP_INFO(get_logger(), "KML: %s", kml.c_str());

            search.FreeSolutionNodes();
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Pathfinding failed");
        }

        return waypoints;
    }

    void onTarget(const GeoPointMsg::SharedPtr msg)
    {
        auto start = Waypoint{currentLocation.latitude, currentLocation.longitude};
        auto end = Waypoint{msg->latitude, msg->longitude};

        auto waypoints = pathfind(start, end);

        PlanMsg planMsg;
        planMsg.waypoints = std::vector<GeoPointMsg>();

        for (const auto &waypoint : waypoints)
        {
            GeoPointMsg geoPointMsg;
            geoPointMsg.latitude = waypoint.first;
            geoPointMsg.longitude = waypoint.second;
            planMsg.waypoints.push_back(geoPointMsg);
        }

        planPublisher->publish(planMsg);
    }

public:
    PathFinder() : Node("pathfinder")
    {
        using namespace std::placeholders;

        currentLocation.latitude = 38.407241;
        currentLocation.longitude = -110.790854;

        planPublisher = create_publisher<PlanMsg>("/pathfinder/plan", 10);
        targetSubscriber = create_subscription<GeoPointMsg>(
            "/pathfinder/target",
            10,
            std::bind(&PathFinder::onTarget, this, _1));
    }
};

int main(int argc, char *argv[])
{
    init(argc, argv);

    unsigned char *inputData = loadImage();

    for (int i = 0; i < width * height; i++)
    {
        map[i] = inputData[i] > 0;
    }

    stbi_image_free(inputData);

    spin(std::make_shared<PathFinder>());

    shutdown();

    return 0;
}