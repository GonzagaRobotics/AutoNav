#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "auto_nav_interfaces/Types.hpp"

class AutoNav : public rclcpp::Node
{
private:
    State state;

    rclcpp::Publisher<State>::SharedPtr statePub;

    rclcpp::Service<QueryStateService>::SharedPtr queryStateService;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enableSub;
    rclcpp::Subscription<Instruction>::SharedPtr instructionSub;

    void queryState(const std::shared_ptr<QueryStateService::Request> request,
                    std::shared_ptr<QueryStateService::Response> response);

    void onEnable(const std_msgs::msg::Bool::SharedPtr msg);
    void onInstruction(const Instruction msg);

    void setState(State newState);

public:
    AutoNav();
};