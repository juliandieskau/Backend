#pragma once

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

    Position2d get_position();
    static Waypoint from_ros(const ros_t &);
    static ros_t to_ros(const Waypoint &);

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
