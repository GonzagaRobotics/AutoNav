#include "pathfinder.hpp"

void Pathfinder::onPathfinderCheck()
{
    if (!pathfinding)
    {
        return;
    }

    if (pathfinderFuture.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
    {
        auto result = pathfinderFuture.get();

        auto resultMsg = std::make_shared<MakePlan::Result>();
        for (auto &waypoint : result.first)
        {
            auto geoLocMsg = auto_nav_interfaces::msg::GeoLoc();
            geoLocMsg.latitude = waypoint.latitude;
            geoLocMsg.longitude = waypoint.longitude;

            resultMsg->plan.waypoints.push_back(geoLocMsg);
        }

        if (result.second.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Pathfinding complete");

#ifdef DEBUG
            auto static_dir = this->get_parameter("static_dir").as_string();

            GeoLoc currentLocation;
            currentLocation.latitude = currentGoalHandle->get_goal()->current_location.latitude;
            currentLocation.longitude = currentGoalHandle->get_goal()->current_location.longitude;

            debugKML(static_dir, currentLocation, Plan{result.first});
#endif

            currentGoalHandle->succeed(resultMsg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Pathfinding failed: %s", result.second.c_str());

            currentGoalHandle->abort(resultMsg);
        }

        pathfinding = false;
    }
}

rclcpp_action::GoalResponse Pathfinder::onMakePlanGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MakePlan::Goal>)
{
    RCLCPP_INFO(this->get_logger(), "Received a new goal");

    if (pathfinding)
    {
        RCLCPP_WARN(this->get_logger(), "Canceling current pathfinding to start a new one");

        pathfinding = false;
        pathfinderFuture.wait();
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Pathfinder::onMakePlanCancel(const std::shared_ptr<MakePlanGoalHandle>)
{
    RCLCPP_INFO(this->get_logger(), "Goal canceled");

    pathfinding = false;
    pathfinderFuture.wait();

    return rclcpp_action::CancelResponse::ACCEPT;
}

void Pathfinder::onMakePlanExecute(const std::shared_ptr<MakePlanGoalHandle> goalHandle)
{
    auto goalMsg = goalHandle->get_goal();

    GeoLoc currentLocation;
    currentLocation.latitude = goalMsg->current_location.latitude;
    currentLocation.longitude = goalMsg->current_location.longitude;

    Target target;
    target.type = static_cast<TargetType>(goalMsg->target.type);
    target.location.latitude = goalMsg->target.location.latitude;
    target.location.longitude = goalMsg->target.location.longitude;

    RCLCPP_INFO(this->get_logger(), "Executing goal from %s to %s",
                currentLocation.to_string().c_str(),
                target.to_string().c_str());

    currentGoalHandle = goalHandle;
    pathfinding = true;

    pathfinderFuture = std::async(
        std::launch::async,
        [this, currentLocation, target]()
        {
            Search search(this->site);

            return search.findPath(currentLocation, target.location, pathfinding);
        });
}

Pathfinder::Pathfinder() : Node("pathfinder")
{
    using namespace std::placeholders;

    auto static_dir = this->declare_parameter("static_dir", rclcpp::PARAMETER_STRING);

    // Double check that the static directory ends with a slash
    if (static_dir.get<std::string>().back() != '/')
    {
        throw std::runtime_error("static_dir must end with a slash");
    }

    auto site_name = this->declare_parameter("site_name", rclcpp::PARAMETER_STRING);

    // Now that we have the directory and name, we can load the site
    SiteLoader loader(static_dir.get<std::string>());

    this->site = loader.load(site_name.get<std::string>());

    pathfinderCheckTimer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Pathfinder::onPathfinderCheck, this));

    this->makePlanServer = rclcpp_action::create_server<MakePlan>(
        this,
        "/make_plan",
        std::bind(&Pathfinder::onMakePlanGoal, this, _1, _2),
        std::bind(&Pathfinder::onMakePlanCancel, this, _1),
        std::bind(&Pathfinder::onMakePlanExecute, this, _1));

    RCLCPP_INFO(this->get_logger(), "Pathfinder ready");
}

Pathfinder::~Pathfinder()
{
    // If we are pathfinding, we need to stop
    if (pathfinding)
    {
        RCLCPP_WARN(this->get_logger(), "Canceling current pathfinding due to shutdown");

        pathfinding = false;
        pathfinderFuture.wait();
    }
}
