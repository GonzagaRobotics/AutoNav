#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "auto_nav_interfaces/Types.hpp"

class AutoNav : public rclcpp::Node
{
private:
    State state;
    GeoLoc currentLocation;
    std::optional<Target> target;
    std::optional<Plan> plan;
    /** The state to return to once resumed. */
    std::optional<State> pausedState;

    rclcpp::Publisher<State>::SharedPtr statePub;
    rclcpp::Publisher<Plan>::SharedPtr planPub;

    rclcpp::Service<QueryStateService>::SharedPtr queryStateService;

    rclcpp_action::Client<MakePlan>::SharedPtr makePlanClient;

    rclcpp::Subscription<Target>::SharedPtr targetSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enableSub;
    rclcpp::Subscription<Instruction>::SharedPtr instructionSub;

    void queryState(const std::shared_ptr<QueryStateService::Request> request,
                    std::shared_ptr<QueryStateService::Response> response);

    void onMakePlanGoalResponse(const MakePlanCGH::SharedPtr &goalHandle);

    void onMakePlanFeedback(MakePlanCGH::SharedPtr, const MakePlanFeedback::ConstSharedPtr) {}

    void onMakePlanResult(const MakePlanCGH::WrappedResult &result);

    void onTarget(const Target msg);
    void onEnable(const std_msgs::msg::Bool::SharedPtr msg);
    void onInstruction(const Instruction msg);

    void setState(State newState);
    void resetPlans();

public:
    AutoNav();
};