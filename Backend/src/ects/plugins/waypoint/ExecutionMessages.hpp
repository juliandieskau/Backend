#pragma once
/* ECTS - WaypointExecutionMessages.hpp
 * Message definitions relating to the execution of waypoint lists.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

#include "ects/EmptyMessage.hpp"
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

namespace ects::plugins::waypoints {

using EmptyStartMessage = EmptyMessage<std_msgs::Empty>;
using EmptyStopMessage = EmptyMessage<std_msgs::Empty>;
class IsExecutingMessage {
  public:
    using ros_t = std_msgs::Bool;
    using to_ros_t = ros_t;

    IsExecutingMessage(bool is_executing) : is_executing(is_executing) {}
    static auto to_ros(const IsExecutingMessage &m) {
        ros_t r;
        r.data = m.is_executing;
        return r;
    }

  private:
    bool is_executing;
};
} // namespace ects::plugins::waypoints