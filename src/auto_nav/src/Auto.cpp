#include "Auto.h"

AutoNav::AutoNav() : Node("auto_nav")
{
    using namespace std::placeholders;

    state = State::Disabled;
    target.reset();
    plan.reset();

    currentLocation.latitude = 38.407241;
    currentLocation.longitude = -110.790854;

    killswitchSubscriber = create_subscription<Killswitch>(
        "/killswitch",
        10,
        std::bind(&AutoNav::onKillswitch, this, _1));

    confirmStartSubscriber = create_subscription<ConfirmStart>(
        "/confirm_start",
        10,
        std::bind(&AutoNav::onConfirmStart, this, _1));

    confirmStopSubscriber = create_subscription<ConfirmStop>(
        "/confirm_stop",
        10,
        std::bind(&AutoNav::onConfirmStop, this, _1));

    statePublisher = create_publisher<State>("/auto_nav/state", 10);
    planPublisher = create_publisher<Plan>("/auto_nav/plan", 10);

    targetSubscriber = create_subscription<Target>(
        "/auto_nav/target",
        10,
        std::bind(&AutoNav::onTarget, this, _1));

    instructionSubscriber = create_subscription<Instruction>(
        "/auto_nav/instruction",
        10,
        std::bind(&AutoNav::onInstruction, this, _1));

    pathfinderTargetPublisher = create_publisher<Target>("/pathfinder/target", 10);

    pathfinderPlanSubscriber = create_subscription<Plan>(
        "/pathfinder/plan",
        10,
        std::bind(&AutoNav::onPathfinderPlan, this, _1));
}

void AutoNav::onKillswitch(const Killswitch)
{
    state = State::Disabled;
    target.reset();
    plan.reset();

    publishStatus();
}

void AutoNav::onConfirmStart(const ConfirmStart)
{
    state = State::Ready;
    publishStatus();
}

void AutoNav::onConfirmStop(const ConfirmStop)
{
    state = State::Disabled;
    target.reset();
    plan.reset();

    publishStatus();
}

void AutoNav::publishStatus()
{
    statePublisher->publish(state);
    planPublisher->publish(plan.value_or(Plan()));
}

void AutoNav::onTarget(const Target msg)
{
    target = msg;
    state = State::Planning;

    publishStatus();

    pathfinderTargetPublisher->publish(msg);
}

void AutoNav::onInstruction(const Instruction msg)
{
    if (state == State::Ready)
    {
        if (msg == Instruction::Execute)
        {
            state = State::Traveling;
        }
        else if (msg == Instruction::Terminate)
        {
            state = State::Ready;
        }
    }

    publishStatus();
}

void AutoNav::onPathfinderPlan(const PlanMsg::SharedPtr msg)
{
    plan = Plan{
        Target{
            GeoPoint{msg->target.location.latitude, msg->target.location.longitude},
            static_cast<TargetType>(msg->target.type.type)},
        std::vector<GeoPoint>()};

    for (const auto &waypoint : msg->waypoints)
    {
        plan->waypoints.push_back(GeoPoint{waypoint.latitude, waypoint.longitude});
    }

    state = State::WAITING;

    publishStatus();
}