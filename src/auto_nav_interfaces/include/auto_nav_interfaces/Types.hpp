#pragma once

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "auto_nav_interfaces/msg/instruction.hpp"
#include "auto_nav_interfaces/msg/state.hpp"
#include "auto_nav_interfaces/msg/geo_loc.hpp"
#include "auto_nav_interfaces/msg/target.hpp"
#include "auto_nav_interfaces/msg/plan.hpp"
#include "auto_nav_interfaces/srv/query_state.hpp"
#include "auto_nav_interfaces/action/make_plan.hpp"

#define TYPE_ADAPTER_OPEN(custom, ros)         \
    template <>                                \
    struct rclcpp::TypeAdapter<custom, ros>    \
    {                                          \
        using is_specialized = std::true_type; \
        using custom_type = custom;            \
        using ros_message_type = ros;

#define TYPE_ADAPTER_CLOSE() \
    }                        \
    ;

#define TO_ROS_OPEN()                                                                            \
    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination) \
    {

#define TO_ROS_CLOSE() \
    }

#define TO_CUSTOM_OPEN()                                                                    \
    static void convert_to_custom(const ros_message_type &source, custom_type &destination) \
    {
#define TO_CUSTOM_CLOSE() \
    }

/**
 * The types of targets that AutoNav can navigate to.
 */
enum class TargetType : uint8_t
{
    /** High precision GPS coordinates. */
    GEO_LOC,
    /** Post marked with ARUCO tags. */
    ARUCO,
    /** Rubber mallet. */
    MALLET,
    /** Water bottle. */
    BOTTLE
};

/**
 * The instructions that can be sent to AutoNav.
 */
enum class Instruction : uint8_t
{
    /** Temporarily stop the current plan. */
    PAUSE,
    /** Continue following the current plan. */
    RESUME,
    /** Begin executing the current plan. */
    EXECUTE,
    /** Stop the current plan and return to the READY state. */
    TERMINATE
};

/**
 * The possible states of AutoNav.
 */
enum class State : uint8_t
{
    /** AutoNav is disabled and won't do anything. */
    DISABLED,
    /** AutoNav is ready to receive a target. */
    READY,
    /** AutoNav is making a plan to reach the target. */
    PLANNING,
    /** AutoNav has a plan and is waiting for an instruction. */
    WAITING,
    /** AutoNav is traveling towards the target. */
    TRAVELING,
    /** AutoNav is searching for the target. */
    TERMINAL_SEARCHING,
    /** AutoNav found the target and is moving towards it. */
    TERMINAL_MOVING,
    /** AutoNav is paused. */
    PAUSED,
    /** AutoNav reached the target. */
    SUCCESS,
    /** AutoNav is unable to reach the target. */
    FAILURE
};

/**
 * A high precision GPS location.
 */
struct GeoLoc
{
    using SharedPtr = std::shared_ptr<GeoLoc>;

    /** -90 to 90 degrees. */
    double latitude;
    /** -180 to 180 degrees. */
    double longitude;

    std::string to_string() const
    {
        return "(" + std::to_string(latitude) + ", " + std::to_string(longitude) + ")";
    }
};

/**
 * A target that AutoNav can navigate to.
 */
struct Target
{
    using SharedPtr = std::shared_ptr<Target>;

    /** Where the target is located. */
    GeoLoc location;
    /** The type of target. */
    TargetType type;

    std::string to_string() const
    {
        std::string typeString;

        switch (type)
        {
        case TargetType::GEO_LOC:
            typeString = "Geo Location";
            break;
        case TargetType::ARUCO:
            typeString = "ARUCO Post";
            break;
        case TargetType::MALLET:
            typeString = "Rubber Mallet";
            break;
        case TargetType::BOTTLE:
            typeString = "Water Bottle";
            break;
        }

        return typeString + " at " + location.to_string();
    }
};

/**
 * A plan to navigate to a target.
 */
struct Plan
{
    using SharedPtr = std::shared_ptr<Plan>;

    /** The waypoints to reach the target. The final element is the target. */
    std::vector<GeoLoc> waypoints;
};

using MakePlan = auto_nav_interfaces::action::MakePlan;
using MakePlanGoalHandle = rclcpp_action::ServerGoalHandle<MakePlan>;

using QueryStateService = auto_nav_interfaces::srv::QueryState;
using QueryStateRequest = auto_nav_interfaces::srv::QueryState::Request;
using QueryStateResponse = auto_nav_interfaces::srv::QueryState::Response;

TYPE_ADAPTER_OPEN(Instruction, auto_nav_interfaces::msg::Instruction)
TO_ROS_OPEN()
destination.instruction = static_cast<uint8_t>(source);
TO_ROS_CLOSE()

TO_CUSTOM_OPEN()
destination = static_cast<Instruction>(source.instruction);
TO_CUSTOM_CLOSE()
TYPE_ADAPTER_CLOSE()

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Instruction, auto_nav_interfaces::msg::Instruction);

TYPE_ADAPTER_OPEN(State, auto_nav_interfaces::msg::State)
TO_ROS_OPEN()
destination.state = static_cast<uint8_t>(source);
TO_ROS_CLOSE()

TO_CUSTOM_OPEN()
destination = static_cast<State>(source.state);
TO_CUSTOM_CLOSE()
TYPE_ADAPTER_CLOSE()

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(State, auto_nav_interfaces::msg::State);

TYPE_ADAPTER_OPEN(GeoLoc, auto_nav_interfaces::msg::GeoLoc)
TO_ROS_OPEN()
destination.latitude = source.latitude;
destination.longitude = source.longitude;
TO_ROS_CLOSE()

TO_CUSTOM_OPEN()
destination.latitude = source.latitude;
destination.longitude = source.longitude;
TO_CUSTOM_CLOSE()
TYPE_ADAPTER_CLOSE()

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(GeoLoc, auto_nav_interfaces::msg::GeoLoc);

TYPE_ADAPTER_OPEN(Target, auto_nav_interfaces::msg::Target)
TO_ROS_OPEN()
rclcpp::TypeAdapter<GeoLoc, auto_nav_interfaces::msg::GeoLoc>::
    convert_to_ros_message(source.location, destination.location);

destination.type = static_cast<uint8_t>(source.type);
TO_ROS_CLOSE()

TO_CUSTOM_OPEN()
rclcpp::TypeAdapter<GeoLoc, auto_nav_interfaces::msg::GeoLoc>::
    convert_to_custom(source.location, destination.location);

destination.type = static_cast<TargetType>(source.type);
TO_CUSTOM_CLOSE()
TYPE_ADAPTER_CLOSE()

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Target, auto_nav_interfaces::msg::Target);

TYPE_ADAPTER_OPEN(Plan, auto_nav_interfaces::msg::Plan)
TO_ROS_OPEN()
for (const auto &waypoint : source.waypoints)
{
    auto_nav_interfaces::msg::GeoLoc geo_loc;
    rclcpp::TypeAdapter<GeoLoc, auto_nav_interfaces::msg::GeoLoc>::
        convert_to_ros_message(waypoint, geo_loc);

    destination.waypoints.push_back(geo_loc);
}
TO_ROS_CLOSE()

TO_CUSTOM_OPEN()
for (const auto &geo_loc : source.waypoints)
{
    GeoLoc waypoint;
    rclcpp::TypeAdapter<GeoLoc, auto_nav_interfaces::msg::GeoLoc>::
        convert_to_custom(geo_loc, waypoint);

    destination.waypoints.push_back(waypoint);
}
TO_CUSTOM_CLOSE()
TYPE_ADAPTER_CLOSE()

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Plan, auto_nav_interfaces::msg::Plan);