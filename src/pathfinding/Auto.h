#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rcs_interfaces/msg/killswitch.hpp"
#include "rcs_interfaces/msg/confirm_start.hpp"
#include "rcs_interfaces/msg/confirm_stop.hpp"
#include "auto_nav_msgs/msg/geo_point.hpp"
#include "auto_nav_msgs/msg/target_type.hpp"
#include "auto_nav_msgs/msg/target.hpp"
#include "auto_nav_msgs/msg/state.hpp"
#include "auto_nav_msgs/msg/plan.hpp"
#include "auto_nav_msgs/msg/instruction.hpp"

using EmptyMsg = std_msgs::msg::Empty;
using KillswitchMsg = rcs_interfaces::msg::Killswitch;
using ConfirmStartMsg = rcs_interfaces::msg::ConfirmStart;
using ConfirmStopMsg = rcs_interfaces::msg::ConfirmStop;

using GeoPointMsg = auto_nav_msgs::msg::GeoPoint;
using TargetTypeMsg = auto_nav_msgs::msg::TargetType;
using TargetMsg = auto_nav_msgs::msg::Target;
using StateMsg = auto_nav_msgs::msg::State;
using PlanMsg = auto_nav_msgs::msg::Plan;
using InstructionMsg = auto_nav_msgs::msg::Instruction;

struct GeoPoint
{
    double latitude;
    double longitude;
};

enum class TargetType
{
    GEO_POINT,
    POST,
    MALLET,
    BOTTLE,
    NULL_TARGET
};

struct Target
{
    GeoPoint location;
    TargetType type;
};

enum class State
{
    DISABLED,
    WAITING,
    PLANNING,
    MOVING,
    TERMINAL,
    SUCCESS
};

enum class Instruction
{
    PAUSE,
    RESUME,
    TERMINATE,
    EXECUTE
};

struct Plan
{
    Target target;
    std::vector<GeoPoint> waypoints;
};

class AutoNav : public rclcpp::Node
{
private:
    State state;
    GeoPoint currentLocation;
    std::optional<Target> target;
    std::optional<Plan> plan;

    void onKillswitch(const KillswitchMsg::SharedPtr);
    void onConfirmStart(const ConfirmStartMsg::SharedPtr);
    void onConfirmStop(const ConfirmStopMsg::SharedPtr);

    rclcpp::Subscription<KillswitchMsg>::SharedPtr killswitchSubscriber;
    rclcpp::Subscription<ConfirmStartMsg>::SharedPtr confirmStartSubscriber;
    rclcpp::Subscription<ConfirmStopMsg>::SharedPtr confirmStopSubscriber;

    void publishStatus();
    void onQuery(const EmptyMsg::SharedPtr);
    void onTarget(const TargetMsg::SharedPtr msg);
    void onInstruction(const InstructionMsg::SharedPtr msg);

    rclcpp::Publisher<StateMsg>::SharedPtr statePublisher;
    rclcpp::Publisher<PlanMsg>::SharedPtr planPublisher;
    rclcpp::Subscription<EmptyMsg>::SharedPtr querySubscriber;
    rclcpp::Subscription<TargetMsg>::SharedPtr targetSubscriber;
    rclcpp::Subscription<InstructionMsg>::SharedPtr instructionSubscriber;

    void onPathfinderPlan(const PlanMsg::SharedPtr msg);

    rclcpp::Publisher<GeoPointMsg>::SharedPtr pathfinderTargetPublisher;
    rclcpp::Subscription<PlanMsg>::SharedPtr pathfinderPlanSubscriber;

public:
    AutoNav();
};