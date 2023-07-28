#pragma once

#include "JsonHelper.hpp"
#include "ects/Common.hpp"
#include "ects/Waypoint.h"
#include <chrono>

namespace ects::plugins::waypoints {
using common::Position2d, common::Heading2d;

class Waypoint {
  public:
    using ros_t = ects::Waypoint;
    using from_ros_t = ros_t;
    using to_ros_t = ros_t;

    Waypoint() = default;
    auto get_position() -> Position2d;
    auto get_name() -> std::string;
    static auto from_ros(const ros_t &) -> Waypoint;
    static auto to_ros(const Waypoint &) -> ros_t;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Waypoint, name, position, heading,
                                   wait_time);

  private:
    Waypoint(std::string name, Position2d position,
             std::optional<Heading2d> heading,
             std::chrono::duration<double> wait_time);
    std::string name;
    Position2d position;
    std::optional<Heading2d> heading;
    std::chrono::duration<double> wait_time;
};

} // namespace ects::plugins::waypoints
