#pragma once
/* ECTS - Waypoint.hpp
 * Class desribing a waypoint in the waypoint mission of the robot. This
 * waypoint has more data (a name), than the IOSB waypoint used by the robot.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

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
    auto get_position() const -> Position2d;
    auto get_name() const -> std::string;
    auto get_heading() const -> std::optional<Heading2d>;
    auto get_wait_time() const -> std::chrono::duration<double>;
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
