#include "auto_nav.hpp"

AutoNav::AutoNav() : Node("auto_nav")
{
    using namespace std::placeholders;

    state = State::DISABLED;
    target.reset();
    plan.reset();

    currentLocation.latitude = 38.407241;
    currentLocation.longitude = -110.790854;

    statePublisher = create_publisher<State>("/auto_nav/state", 10);

    enableSubscription = create_subscription<std_msgs::msg::Bool>(
        "/auto_nav/enable", 10, std::bind(&AutoNav::enableCallback, this, _1));

    instructionSubscription = create_subscription<Instruction>(
        "/auto_nav/instruction", 10, std::bind(&AutoNav::instructionCallback, this, _1));

    targetSubscription = create_subscription<Target>(
        "/auto_nav/target", 10, std::bind(&AutoNav::targetCallback, this, _1));

    planPublisher = create_publisher<Plan>("/auto_nav/plan", 10);

    makePlanClient = rclcpp_action::create_client<MakePlan>(this, "/make_plan");

    RCLCPP_INFO(get_logger(), "AutoNav Ready");
}

void AutoNav::enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        if (state == State::DISABLED)
        {
            RCLCPP_INFO(get_logger(), "Enabling");
            state = State::READY;
        }
    }
    else
    {
        if (state == State::READY)
        {
            RCLCPP_INFO(get_logger(), "Disabling");
            state = State::DISABLED;
        }

        // TODO: Cancel any ongoing navigation
    }

    statePublisher->publish(state);
}

void AutoNav::instructionCallback(const Instruction msg)
{
    if (state == State::WAITING)
    {
        if (msg == Instruction::EXECUTE)
        {
            RCLCPP_INFO(get_logger(), "Executing plan");
            state = State::TRAVELING;
        }
        else if (msg == Instruction::TERMINATE)
        {
            RCLCPP_INFO(get_logger(), "Terminating plan");
            target.reset();
            plan.reset();
            state = State::READY;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Got invalid instruction %d while waiting", (int)msg);
        }
    }
    else if (isStateMoving())
    {
        if (msg == Instruction::PAUSE)
        {
            RCLCPP_INFO(get_logger(), "Pausing plan");
            state = State::PAUSED;
        }
        else if (msg == Instruction::TERMINATE)
        {
            RCLCPP_INFO(get_logger(), "Terminating plan");
            target.reset();
            plan.reset();
            state = State::READY;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Got invalid instruction %d while moving", (int)msg);
        }
    }
    else if (state == State::PAUSED)
    {
        if (msg == Instruction::RESUME)
        {
            RCLCPP_INFO(get_logger(), "Resuming plan");
            state = State::TRAVELING;
        }
        else if (msg == Instruction::TERMINATE)
        {
            RCLCPP_INFO(get_logger(), "Terminating plan");
            target.reset();
            plan.reset();
            state = State::READY;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Got invalid instruction %d while paused", (int)msg);
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Got unexpected instruction %d while in state %d", (int)msg, (int)state);
    }

    statePublisher->publish(state);
}

void AutoNav::targetCallback(const Target::SharedPtr msg)
{
    using namespace std::placeholders;

    if (state == State::READY)
    {
        RCLCPP_INFO(get_logger(), "Got target");
        target = *msg;
        state = State::PLANNING;

        if (!makePlanClient->wait_for_action_server())
        {
            state = State::FAILURE;
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        }

        auto goalOpts = rclcpp_action::Client<MakePlan>::SendGoalOptions();
        goalOpts.goal_response_callback = std::bind(&AutoNav::onMakePlanGoalResponse, this, _1);
        goalOpts.result_callback = std::bind(&AutoNav::onMakePlanResult, this, _1);

        auto targetMsg = MakePlan::Goal();
        targetMsg.current_location.latitude = currentLocation.latitude;
        targetMsg.current_location.longitude = currentLocation.longitude;
        targetMsg.target.location.latitude = msg->location.latitude;
        targetMsg.target.location.longitude = msg->location.longitude;
        targetMsg.target.type = (int)msg->type;

        makePlanClient->async_send_goal(targetMsg, goalOpts);

        statePublisher->publish(state);
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Got target while in state %d", (int)state);
    }
}

void AutoNav::onMakePlanGoalResponse(const rclcpp_action::ClientGoalHandle<MakePlan>::SharedPtr &goalHandle)
{
    if (!goalHandle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
    }
}

void AutoNav::onMakePlanResult(const rclcpp_action::ClientGoalHandle<MakePlan>::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
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
    state = State::WAITING;
    statePublisher->publish(state);
    planPublisher->publish(plan);
}
