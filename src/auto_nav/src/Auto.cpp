#include "Auto.h"

AutoNav::AutoNav() : Node("auto_nav")
{
    using namespace std::placeholders;

    state = State::DISABLED;
    target.reset();
    plan.reset();

    currentLocation.latitude = 38.407241;
    currentLocation.longitude = -110.790854;

    killswitchSubscriber = create_subscription<KillswitchMsg>(
        "/killswitch",
        10,
        std::bind(&AutoNav::onKillswitch, this, _1));

    confirmStartSubscriber = create_subscription<ConfirmStartMsg>(
        "/confirm_start",
        10,
        std::bind(&AutoNav::onConfirmStart, this, _1));

    confirmStopSubscriber = create_subscription<ConfirmStopMsg>(
        "/confirm_stop",
        10,
        std::bind(&AutoNav::onConfirmStop, this, _1));

    statePublisher = create_publisher<StateMsg>("/auto_nav/state", 10);
    planPublisher = create_publisher<PlanMsg>("/auto_nav/plan", 10);

    querySubscriber = create_subscription<EmptyMsg>(
        "/auto_nav/query",
        10,
        std::bind(&AutoNav::onQuery, this, _1));

    targetSubscriber = create_subscription<TargetMsg>(
        "/auto_nav/target",
        10,
        std::bind(&AutoNav::onTarget, this, _1));

    instructionSubscriber = create_subscription<InstructionMsg>(
        "/auto_nav/instruction",
        10,
        std::bind(&AutoNav::onInstruction, this, _1));

    pathfinderTargetPublisher = create_publisher<GeoPointMsg>("/pathfinder/target", 10);

    pathfinderPlanSubscriber = create_subscription<PlanMsg>(
        "/pathfinder/plan",
        10,
        std::bind(&AutoNav::onPathfinderPlan, this, _1));
}

void AutoNav::onKillswitch(const KillswitchMsg::SharedPtr)
{
    state = State::DISABLED;
    target.reset();
    plan.reset();

    publishStatus();
}

void AutoNav::onConfirmStart(const ConfirmStartMsg::SharedPtr)
{
    publishStatus();
}

void AutoNav::onConfirmStop(const ConfirmStopMsg::SharedPtr)
{
    state = State::DISABLED;
    target.reset();
    plan.reset();

    publishStatus();
}

void AutoNav::publishStatus()
{
    StateMsg stateMsg;
    stateMsg.state = static_cast<uint16_t>(state);

    PlanMsg planMsg;
    planMsg.waypoints = std::vector<GeoPointMsg>();
    if (plan)
    {
        for (const auto &waypoint : plan->waypoints)
        {
            GeoPointMsg geoPointMsg;
            geoPointMsg.latitude = waypoint.latitude;
            geoPointMsg.longitude = waypoint.longitude;
            planMsg.waypoints.push_back(geoPointMsg);
        }
    }

    TargetMsg targetMsg;
    if (target)
    {
        targetMsg.location.latitude = target->location.latitude;
        targetMsg.location.longitude = target->location.longitude;
        targetMsg.type.type = static_cast<uint16_t>(target->type);
    }
    else
    {
        targetMsg.type.type = static_cast<uint16_t>(TargetType::NULL_TARGET);
    }

    planMsg.target = targetMsg;

    statePublisher->publish(stateMsg);
    planPublisher->publish(planMsg);
}

void AutoNav::onQuery(const EmptyMsg::SharedPtr)
{
    publishStatus();
}

void AutoNav::onTarget(const TargetMsg::SharedPtr msg)
{
    target = Target{
        GeoPoint{msg->location.latitude, msg->location.longitude},
        static_cast<TargetType>(msg->type.type)};

    state = State::PLANNING;

    publishStatus();

    pathfinderTargetPublisher->publish(msg->location);
}

void AutoNav::onInstruction(const InstructionMsg::SharedPtr msg)
{
    switch (static_cast<Instruction>(msg->instruction))
    {
    case Instruction::PAUSE:
        // Pause the current plan
        state = State::WAITING;
        break;
    case Instruction::RESUME:
        // TODO: Resume from pause
        state = State::MOVING;
        break;
    case Instruction::TERMINATE:
        // Terminate the current plan
        state = State::DISABLED;
        target.reset();
        plan.reset();
        break;
    case Instruction::EXECUTE:
        // Execute the current plan
        state = State::MOVING;
        break;
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