#pragma once

#include <vector>
#include <utility>
#include <thread>
#include <functional>
#include <optional>
#include <future>
#include <atomic>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "site.hpp"
#include "site_loader.hpp"
#include "search.hpp"

#ifdef DEBUG
#include "debug/debug_kml.hpp"
#endif

/**
 * Finds paths through a site.
 */
class Pathfinder : public rclcpp::Node
{
private:
    /** The site we are currently on. */
    std::shared_ptr<Site> site;

    /** Are we currently pathfinding? */
    std::atomic<bool> pathfinding;

    /** The future for the pathfinder's search. */
    std::future<std::pair<std::vector<GeoLoc>, std::string>> pathfinderFuture;

    /** The current goal handle for making a plan. */
    std::shared_ptr<MakePlanGoalHandle> currentGoalHandle;

    /** The action server for making plans. */
    rclcpp_action::Server<MakePlan>::SharedPtr makePlanServer;

    /** The timer that checks if pathfinding is complete. */
    rclcpp::TimerBase::SharedPtr pathfinderCheckTimer;

    void onPathfinderCheck();

    rclcpp_action::GoalResponse onMakePlanGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MakePlan::Goal> goal);

    rclcpp_action::CancelResponse onMakePlanCancel(
        const std::shared_ptr<MakePlanGoalHandle> goalHandle);

    void onMakePlanExecute(
        const std::shared_ptr<MakePlanGoalHandle> goalHandle);

public:
    Pathfinder();

    ~Pathfinder();
};