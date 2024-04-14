#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcs_interfaces/Types.h"
#include "auto_nav_interfaces/Types.h"

class AutoNav : public rclcpp::Node
{
private:
    State state;
    GeoLoc currentLocation;

    std::optional<Target> target;
    std::optional<Plan> plan;

    void onKillswitch(const Killswitch);
    void onConfirmStart(const ConfirmStart);
    void onConfirmStop(const ConfirmStop);

    rclcpp::Subscription<Killswitch>::SharedPtr killswitchSubscriber;
    rclcpp::Subscription<ConfirmStart>::SharedPtr confirmStartSubscriber;
    rclcpp::Subscription<ConfirmStop>::SharedPtr confirmStopSubscriber;

    void publishStatus();
    void onTarget(const Target);
    void onInstruction(const Instruction);

    rclcpp::Publisher<State>::SharedPtr statePublisher;
    rclcpp::Publisher<Plan>::SharedPtr planPublisher;
    rclcpp::Subscription<Target>::SharedPtr targetSubscriber;
    rclcpp::Subscription<Instruction>::SharedPtr instructionSubscriber;

    void onPathfinderPlan(const Plan);

    rclcpp::Publisher<Target>::SharedPtr pathfinderTargetPublisher;
    rclcpp::Subscription<Plan>::SharedPtr pathfinderPlanSubscriber;

public:
    AutoNav();
};