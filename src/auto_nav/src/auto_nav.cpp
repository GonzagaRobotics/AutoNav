#include "auto_nav.hpp"

void AutoNav::queryState(const std::shared_ptr<QueryStateService::Request>, std::shared_ptr<QueryStateService::Response> response)
{
    response->state = (int)state;
}

void AutoNav::onEnable(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (state == State::DISABLED && msg->data)
    {
        setState(State::READY);
    }
    else if (state != State::DISABLED && msg->data == false)
    {
        setState(State::DISABLED);
    }
}

void AutoNav::onInstruction(const Instruction msg)
{
    RCLCPP_INFO(get_logger(), "Received instruction: %d", (int)msg);
}

void AutoNav::setState(State newState)
{
    if (newState == state)
    {
        return;
    }

    state = newState;

    RCLCPP_INFO(get_logger(), "State changed to: %d", (int)state);

    statePub->publish(state);
}

AutoNav::AutoNav() : Node("auto_nav")
{
    using namespace std::placeholders;

    statePub = create_publisher<State>(
        "auto_nav/state",
        10);

    queryStateService = create_service<QueryStateService>(
        "auto_nav/query_state",
        std::bind(&AutoNav::queryState, this, _1, _2));

    enableSub = create_subscription<std_msgs::msg::Bool>(
        "auto_nav/enable",
        10,
        std::bind(&AutoNav::onEnable, this, _1));

    instructionSub = create_subscription<Instruction>(
        "auto_nav/instruction",
        10,
        std::bind(&AutoNav::onInstruction, this, _1));

    state = State::DISABLED;

    RCLCPP_INFO(get_logger(), "AutoNav Ready");
}
