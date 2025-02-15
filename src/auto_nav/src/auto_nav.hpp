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

    rclcpp::Publisher<State>::SharedPtr statePublisher;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enableSubscription;
    rclcpp::Subscription<Instruction>::SharedPtr instructionSubscription;

    rclcpp::Subscription<Target>::SharedPtr targetSubscription;
    rclcpp::Publisher<Plan>::SharedPtr planPublisher;

    rclcpp_action::Client<MakePlan>::SharedPtr makePlanClient;

    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void instructionCallback(const Instruction msg);

    void targetCallback(const Target::SharedPtr msg);

    void onMakePlanGoalResponse(const rclcpp_action::ClientGoalHandle<MakePlan>::SharedPtr &goalHandle);

    void onMakePlanFeedback(rclcpp_action::ClientGoalHandle<MakePlan>::SharedPtr,
                            const auto_nav_interfaces::action::MakePlan::Feedback::ConstSharedPtr) {}

    void onMakePlanResult(const rclcpp_action::ClientGoalHandle<MakePlan>::WrappedResult &result);

    bool isStateMoving()
    {
        return state == State::TRAVELING ||
               state == State::TERMINAL_SEARCHING ||
               state == State::TERMINAL_MOVING;
    }

public:
    AutoNav();
};