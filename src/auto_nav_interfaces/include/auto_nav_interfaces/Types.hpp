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
#include "auto_nav_interfaces/action/make_plan.hpp"

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
    /** AutoNav is traveling towards the target. */
    TRAVELING,
    /** AutoNav is searching for the target. */
    TERMINAL_SEARCHING,
    /** AutoNav found the target and is moving towards it. */
    TERMINAL_MOVING,
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

template <>
struct rclcpp::TypeAdapter<Instruction, auto_nav_interfaces::msg::Instruction>
{
    using is_specialized = std::true_type;
    using custom_type = Instruction;
    using ros_message_type = auto_nav_interfaces::msg::Instruction;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        destination.instruction = static_cast<uint8_t>(source);
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        destination = static_cast<Instruction>(source.instruction);
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Instruction, auto_nav_interfaces::msg::Instruction);

template <>
struct rclcpp::TypeAdapter<State, auto_nav_interfaces::msg::State>
{
    using is_specialized = std::true_type;
    using custom_type = State;
    using ros_message_type = auto_nav_interfaces::msg::State;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        destination.state = static_cast<uint8_t>(source);
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        destination = static_cast<State>(source.state);
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(State, auto_nav_interfaces::msg::State);

template <>
struct rclcpp::TypeAdapter<GeoLoc, auto_nav_interfaces::msg::GeoLoc>
{
    using is_specialized = std::true_type;
    using custom_type = GeoLoc;
    using ros_message_type = auto_nav_interfaces::msg::GeoLoc;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        destination.latitude = source.latitude;
        destination.longitude = source.longitude;
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        destination.latitude = source.latitude;
        destination.longitude = source.longitude;
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(GeoLoc, auto_nav_interfaces::msg::GeoLoc);

template <>
struct rclcpp::TypeAdapter<Target, auto_nav_interfaces::msg::Target>
{
    using is_specialized = std::true_type;
    using custom_type = Target;
    using ros_message_type = auto_nav_interfaces::msg::Target;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        rclcpp::TypeAdapter<GeoLoc, auto_nav_interfaces::msg::GeoLoc>::
            convert_to_ros_message(source.location, destination.location);

        destination.type = static_cast<uint8_t>(source.type);
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        rclcpp::TypeAdapter<GeoLoc, auto_nav_interfaces::msg::GeoLoc>::
            convert_to_custom(source.location, destination.location);

        destination.type = static_cast<TargetType>(source.type);
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Target, auto_nav_interfaces::msg::Target);

template <>
struct rclcpp::TypeAdapter<Plan, auto_nav_interfaces::msg::Plan>
{
    using is_specialized = std::true_type;
    using custom_type = Plan;
    using ros_message_type = auto_nav_interfaces::msg::Plan;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        for (const auto &waypoint : source.waypoints)
        {
            auto_nav_interfaces::msg::GeoLoc geo_loc;
            rclcpp::TypeAdapter<GeoLoc, auto_nav_interfaces::msg::GeoLoc>::
                convert_to_ros_message(waypoint, geo_loc);

            destination.waypoints.push_back(geo_loc);
        }
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        for (const auto &geo_loc : source.waypoints)
        {
            GeoLoc waypoint;
            rclcpp::TypeAdapter<GeoLoc, auto_nav_interfaces::msg::GeoLoc>::
                convert_to_custom(geo_loc, waypoint);

            destination.waypoints.push_back(waypoint);
        }
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Plan, auto_nav_interfaces::msg::Plan);