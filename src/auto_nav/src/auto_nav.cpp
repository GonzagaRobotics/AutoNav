#include "auto_nav.hpp"

void AutoNav::queryState(const std::shared_ptr<QueryStateService::Request>, std::shared_ptr<QueryStateService::Response> response)
{
    response->state = (int)state;
}

void AutoNav::onMakePlanGoalResponse(const MakePlanCGH::SharedPtr &goalHandle)
{
    if (goalHandle == nullptr)
    {
        RCLCPP_ERROR(get_logger(), "MakePlan goal was rejected");
        setState(State::FAILURE);
    }
}

void AutoNav::onMakePlanResult(const MakePlanCGH::WrappedResult &result)
{
    if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
        RCLCPP_ERROR(get_logger(), "MakePlan action aborted");
        setState(State::FAILURE);
        return;
    }

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        return;
    }

    auto plan = Plan();

    for (const auto &waypoint : result.result->plan.waypoints)
    {
        GeoLoc geoLoc;
        geoLoc.latitude = waypoint.latitude;
        geoLoc.longitude = waypoint.longitude;

        plan.waypoints.push_back(geoLoc);
    }

    this->plan = plan;

    setState(State::WAITING);
    planPub->publish(plan);
}

void AutoNav::onTarget(const Target msg)
{
    using namespace std::placeholders;

    if ((state != State::READY) && (state != State::PLANNING))
    {
        RCLCPP_WARN(get_logger(), "Received target in invalid state");
        return;
    }

    RCLCPP_INFO(get_logger(), "Received target: %s", msg.to_string().c_str());

    target = msg;

    if (makePlanClient->wait_for_action_server() == false)
    {
        RCLCPP_ERROR(get_logger(), "MakePlan action server not available");
        setState(State::FAILURE);
        return;
    }

    auto goalOpts = rclcpp_action::Client<MakePlan>::SendGoalOptions();
    goalOpts.goal_response_callback = std::bind(&AutoNav::onMakePlanGoalResponse, this, _1);
    goalOpts.result_callback = std::bind(&AutoNav::onMakePlanResult, this, _1);

    // Actions don't support type adapters, so we need to create a new message
    auto targetMsg = MakePlan::Goal();
    targetMsg.current_location.latitude = currentLocation.latitude;
    targetMsg.current_location.longitude = currentLocation.longitude;
    targetMsg.target.location.latitude = msg.location.latitude;
    targetMsg.target.location.longitude = msg.location.longitude;
    targetMsg.target.type = (int)msg.type;

    makePlanClient->async_send_goal(targetMsg, goalOpts);

    setState(State::PLANNING);
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

        // TODO: Cancel any planning
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

    planPub = create_publisher<Plan>(
        "auto_nav/plan",
        10);

    queryStateService = create_service<QueryStateService>(
        "auto_nav/query_state",
        std::bind(&AutoNav::queryState, this, _1, _2));

    makePlanClient = rclcpp_action::create_client<MakePlan>(this, "make_plan");

    targetSub = create_subscription<Target>(
        "auto_nav/target",
        10,
        std::bind(&AutoNav::onTarget, this, _1));

    enableSub = create_subscription<std_msgs::msg::Bool>(
        "auto_nav/enable",
        10,
        std::bind(&AutoNav::onEnable, this, _1));

    instructionSub = create_subscription<Instruction>(
        "auto_nav/instruction",
        10,
        std::bind(&AutoNav::onInstruction, this, _1));

    state = State::DISABLED;

    currentLocation.latitude = 38.407241;
    currentLocation.longitude = -110.790854;

    RCLCPP_INFO(get_logger(), "AutoNav Ready");
}
