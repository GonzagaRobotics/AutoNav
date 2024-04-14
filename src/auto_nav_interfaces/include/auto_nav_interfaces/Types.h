#pragma once

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "auto_nav_interfaces/msg/instruction.hpp"
#include "auto_nav_interfaces/msg/state.hpp"
#include "auto_nav_interfaces/msg/geo_loc.hpp"
#include "auto_nav_interfaces/msg/target.hpp"
#include "auto_nav_interfaces/msg/plan.hpp"

enum class TargetType : unsigned char
{
    GeoLoc,
    Post,
    Mallet,
    Bottle
};

enum class Instruction : unsigned char
{
    Pause,
    Resume,
    Terminate,
    Execute
};

enum class State : unsigned char
{
    Disabled,
    Ready,
    Planning,
    Traveling,
    TerminalSearching,
    TerminalMoving,
    Success,
    Failure
};

struct GeoLoc
{
    double latitude;
    double longitude;
};

struct Target
{
    GeoLoc location;
    TargetType type;
};

struct Plan
{
    std::vector<GeoLoc> waypoints;
};

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