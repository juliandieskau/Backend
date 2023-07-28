#pragma once

#include "Waypoint.hpp"
#include "ects/WaypointList.h"
#include "nlohmann/json.hpp"

#include <vector>

namespace ects::plugins::waypoints {

class Index {
  public:
    Index(size_t);
    size_t get() const;

  private:
    size_t index;
};

class WaypointList {
  public:
    using ros_t = ects::WaypointList;
    using from_ros_t = ros_t;
    using to_ros_t = ros_t;

    WaypointList() = default;
    auto add_waypoint(Waypoint, Index) -> void;
    auto remove_waypoint(Index) -> void;
    auto replace_waypoint(Index, Waypoint) -> void;
    auto reorder_waypoints(std::vector<Index> permutation) -> void;
    auto reverse_waypoints() -> void;
    auto set_repeat(bool) -> void;
    auto total_length() -> double;
    auto size() -> std::size_t;
    auto get_waypoint(Index) -> Waypoint;

    static auto from_ros(const ros_t &) -> WaypointList;
    static auto to_ros(const WaypointList &) -> ros_t;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WaypointList, waypoints, cyclic);

  private:
    auto check_bounds(Index i) -> void;

    std::vector<Waypoint> waypoints;
    bool cyclic;
};

} // namespace ects::plugins::waypoints
