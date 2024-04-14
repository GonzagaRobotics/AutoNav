#define STB_IMAGE_IMPLEMENTATION

#include <vector>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "auto_nav_interfaces/Types.h"
#include "Site.h"
#include "SiteLoader.h"
#include "Search.h"
#include "debug/DebugKML.h"

using namespace rclcpp;

class Pathfinder : public Node
{
private:
    GeoLoc currentLocation;
    std::shared_ptr<Site> site;

    TimerBase::SharedPtr pathfinderCheckTimer;

    Publisher<Plan>::SharedPtr planPublisher;
    Subscription<Target>::SharedPtr targetSubscriber;

    std::atomic<bool> pathfinderRunning = false;
    std::future<std::pair<Plan, std::string>> pathfinderFuture;

    std::pair<Plan, std::string> pathfind(GeoLoc start, GeoLoc end)
    {
        MapNode startNode = MapNode(start, site);
        MapNode endNode = MapNode(end, site);

        // TODO: Have a cleaner way of handling this case
        if (site->isObstacle(startNode.x, startNode.y, true, true) ||
            site->isObstacle(endNode.x, endNode.y, true, true))
        {
            return std::make_pair(Plan(), "Start or end location is an obstacle");
        }

        AStarSearch<MapNode> search(site->getWidth() * site->getHeight());

        search.SetStartAndGoalStates(startNode, endNode);

        unsigned int searchState;

        do
        {
            if (pathfinderRunning.load() == false)
            {
                search.CancelSearch();
                return std::make_pair(Plan(), "Pathfinding cancelled");
            }

            searchState = search.SearchStep();
        } while (searchState == AStarSearch<MapNode>::SEARCH_STATE_SEARCHING);

        auto plan = Plan();

        // Lazy path simplification
        if (searchState == AStarSearch<MapNode>::SEARCH_STATE_SUCCEEDED)
        {
            // Select the first node in the solution
            MapNode *node = search.GetSolutionStart();
            MapNode *targetNode = search.GetSolutionEnd();

            while (true)
            {
                // Step backwards through the solution until we can draw a straight line to the current node,
                // or we reach the node directly after the current node

                while (targetNode->IsAdjacent(*node) == false)
                {
                    if (site->rayCast(std::make_pair(node->x, node->y), std::make_pair(targetNode->x, targetNode->y)) == false)
                    {
                        break;
                    }

                    targetNode = search.GetSolutionPrev();
                }

                // Add the target node to the plan
                plan.waypoints.push_back(site->getGeoLoc(targetNode->x, targetNode->y));

                // Reset the current node to the target node
                node = targetNode;

                // And reset the target node to the end
                targetNode = search.GetSolutionEnd();

                // If we've reached the end of the solution, break
                if (node == targetNode)
                {
                    break;
                }
            }

            search.FreeSolutionNodes();
        }
        else
        {
            return std::make_pair(Plan(), "Pathfinding failed");
        }

        return std::make_pair(plan, "");
    }

    void onCheckPathfinder()
    {
        if (pathfinderRunning.load() == false)
        {
            return;
        }

        if (pathfinderFuture.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
        {
            auto result = pathfinderFuture.get();

            if (result.second.empty())
            {
                RCLCPP_INFO(get_logger(), "Pathfinding succeeded");
                planPublisher->publish(result.first);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Pathfinding failed: %s", result.second.c_str());
            }

            pathfinderRunning.store(false);
        }
    }

    void onTarget(const Target msg)
    {
        // If the pathfinder is already running, cancel it
        if (pathfinderRunning.load())
        {
            RCLCPP_INFO(get_logger(), "Cancelling existing pathfind operation.");
            pathfinderRunning.store(false);
            pathfinderFuture.wait();
        }

        RCLCPP_INFO(get_logger(), "Pathfinding to target: %f, %f", msg.location.latitude, msg.location.longitude);
        pathfinderRunning.store(true);
        pathfinderFuture = std::async(
            std::launch::async,
            [this, msg]()
            {
                return pathfind(currentLocation, msg.location);
            });
    }

public:
    Pathfinder(std::shared_ptr<Site> site) : Node("pathfinder")
    {
        using namespace std::placeholders;

        currentLocation.latitude = 38.4067985;
        currentLocation.longitude = -110.7913061;

        this->site = site;

        pathfinderCheckTimer = create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&Pathfinder::onCheckPathfinder, this));

        planPublisher = create_publisher<Plan>("/pathfinder/plan", 10);
        targetSubscriber = create_subscription<Target>(
            "/pathfinder/target",
            10,
            std::bind(&Pathfinder::onTarget, this, _1));
    }
};

int main(int argc, char *argv[])
{
    init(argc, argv);

    auto loader = SiteLoader("/home/damon/robotics/AutoNav/static/URC");
    auto map = loader.load();

    spin(std::make_shared<Pathfinder>(map));

    shutdown();

    return 0;
}