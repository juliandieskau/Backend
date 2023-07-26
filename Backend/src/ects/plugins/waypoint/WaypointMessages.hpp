#pragma once

#include "Waypoint.hpp"
#include "WaypointList.hpp"
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include "ects/AddWaypoint.h"
#include "ects/EmptyMessage.hpp"
#include "ects/RemoveWaypoint.h"
#include "ects/ReorderWaypoints.h"
#include "ects/ReplaceWaypoint.h"

namespace ects::plugins::waypoints {

class AddWaypointMessage {
  public:
    using ros_t = ects::AddWaypoint;
    using from_ros_t = ros_t;

    auto get_waypoint() -> Waypoint;
    auto get_index() -> Index;
    static auto from_ros(const ros_t &) -> AddWaypointMessage;

  private:
    AddWaypointMessage(Index index, Waypoint waypoint);

    Index index;
    Waypoint waypoint;
};

class RemoveWaypointMessage {
  public:
    using ros_t = ects::RemoveWaypoint;
    using from_ros_t = ros_t;

    auto get_index() -> Index;
    static auto from_ros(const ros_t &) -> RemoveWaypointMessage;

  private:
    RemoveWaypointMessage(Index index);

    Index index;
};

class ReplaceWaypointMessage {
  public:
    using ros_t = ects::ReplaceWaypoint;
    using from_ros_t = ros_t;

    auto get_index() -> Index;
    auto get_waypoint() -> Waypoint;
    static auto from_ros(const ros_t &) -> ReplaceWaypointMessage;

  private:
    ReplaceWaypointMessage(Index index, Waypoint waypoint);

    Index index;
    Waypoint waypoint;
};

class ReorderWaypointsMessage {
  public:
    using ros_t = ects::ReorderWaypoints;
    using from_ros_t = ros_t;

    auto get_permutation() -> std::vector<Index>;
    static auto from_ros(const ros_t &) -> ReorderWaypointsMessage;

  private:
    std::vector<Index> permutation;
};

class RepeatWaypointsMessage {
  public:
    using ros_t = std_msgs::Bool;
    using from_ros_t = ros_t;

    auto get_repeat() const -> bool;
    static auto from_ros(const ros_t &) -> RepeatWaypointsMessage;

  private:
    RepeatWaypointsMessage(bool);

    bool repeat;
};

using ReverseWaypointsMessage = EmptyMessage<std_msgs::Empty>;

} // namespace ects::plugins::waypoints