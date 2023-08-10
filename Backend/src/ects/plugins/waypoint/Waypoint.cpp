#include "Waypoint.hpp"

#include <utility>

namespace ects::plugins::waypoints {

auto Waypoint::get_position() const -> Position2d { return position; }
auto Waypoint::get_name() const -> std::string { return name; }
auto Waypoint::get_heading() const -> std::optional<Heading2d> {
    return heading;
}
auto Waypoint::get_wait_time() const -> std::chrono::duration<double> {
    return wait_time;
}
auto Waypoint::from_ros(const Waypoint::ros_t &ros_input) -> Waypoint {
    auto wait = std::chrono::duration<double>(ros_input.wait_time);
    std::optional<Heading2d> heading = std::nullopt;
    if (ros_input.use_heading)
        heading = {ros_input.pose.theta, ros_input.heading_accuracy};
    auto pos = Position2d(ros_input.pose.x, ros_input.pose.y, ros_input.radius);

    return {ros_input.name, pos, heading, wait};
}
auto Waypoint::to_ros(const Waypoint &wp) -> Waypoint::ros_t {
    Waypoint::ros_t r;
    auto heading = wp.heading.value_or(Heading2d{0, 0});
    Waypoint::ros_t::_pose_type p;
    p.x = wp.position.get_x();
    p.y = wp.position.get_y();
    p.theta = heading.get_heading();
    r.pose = p;
    r.radius = wp.position.get_radius();
    r.use_heading = wp.heading.has_value();
    r.heading_accuracy = wp.heading->get_accuracy();
    r.wait_time = wp.wait_time.count();
    r.name = wp.name;
    return r;
}
Waypoint::Waypoint(std::string name, Position2d position,
                   std::optional<Heading2d> heading,
                   std::chrono::duration<double> wait_time)
    : name(std::move(name)), position(position), heading(heading),
      wait_time(wait_time) {}

} // namespace ects::plugins::waypoints
