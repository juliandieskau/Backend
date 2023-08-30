#include "WaypointList.hpp"

#include "ros/ros.h"
#include <utility>

namespace ects::plugins::waypoints {

Index::Index(size_t index) : index(index) {}
auto Index::get() const -> size_t { return index; }

auto WaypointList::add_waypoint(Waypoint w, Index i) -> void {
    if (i.get() > size()) {
        std::stringstream s;
        s << "index out of bounds [index: " << i.get() << ", size: " << size()
          << "]";
        throw std::runtime_error(s.str());
    }
    waypoints.insert(waypoints.begin() + i.get(), w);
}
auto WaypointList::remove_waypoint(Index i) -> void {
    check_bounds(i);
    waypoints.erase(waypoints.begin() + i.get());
}
auto WaypointList::replace_waypoint(Index i, Waypoint w) -> void {
    check_bounds(i);
    waypoints[i.get()] = std::move(w);
}
auto WaypointList::reorder_waypoints(std::vector<Index> permutation) -> void {
    if (permutation.size() != size()) {
        std::stringstream s;
        s << "permutation size mismatch [permutation size: "
          << permutation.size() << ", waypoints: " << size() << "]";
        throw std::runtime_error(s.str());
    }
    std::vector<std::optional<Waypoint>> reordered(size());
    for (size_t i = 0; i < size(); ++i) {
        Index idx = permutation[i];
        check_bounds(idx);
        if (reordered[idx.get()].has_value()) {
            std::stringstream s;
            s << "permutation has duplicate index " << idx.get();
            throw std::runtime_error(s.str());
        }
        reordered[idx.get()] = waypoints[i];
    }
    for (size_t i = 0; i < size(); ++i)
        waypoints[i] = *reordered[i];
}
auto WaypointList::reverse_waypoints() -> void {
    std::reverse(waypoints.begin(), waypoints.end());
}
auto WaypointList::set_repeat(bool repeat) -> void { cyclic = repeat; }
auto WaypointList::total_length() const -> double {
    // NOTE: this assumes all waypoints are in the same UTM zone
    std::optional<Position2d> prev;
    double distance = 0;
    for (auto &w : waypoints) {
        if (prev.has_value()) {
            double diff_x = w.get_position().get_x() - prev->get_x();
            double diff_y = w.get_position().get_y() - prev->get_y();
            distance += std::sqrt(diff_x * diff_x + diff_y * diff_y);
        }
        prev = w.get_position();
    }
    return distance;
}
auto WaypointList::size() -> std::size_t { return waypoints.size(); }
auto WaypointList::get_waypoint(Index i) -> Waypoint {
    check_bounds(i);
    return waypoints[i.get()];
}
auto WaypointList::from_ros(const WaypointList::ros_t &ros_input)
    -> WaypointList {
    WaypointList list;
    list.set_repeat(ros_input.cyclic);
    for (const auto &ros_wp : ros_input.waypoints)
        list.waypoints.push_back(Waypoint::from_ros(ros_wp));
    return list;
}
auto WaypointList::to_ros(const WaypointList &list) -> WaypointList::ros_t {
    WaypointList::ros_t r{};
    r.cyclic = list.cyclic;
    for (const auto &wp : list.waypoints)
        r.waypoints.push_back(Waypoint::to_ros(wp));
    r.total_length = list.total_length();
    return r;
}
auto WaypointList::check_bounds(Index i) -> void {
    if (i.get() >= size()) {
        std::stringstream s;
        s << "index out of bounds [index: " << i.get() << ", size: " << size()
          << "]";
        throw std::runtime_error(s.str());
    }
}

auto IOSBWaypointList::to_ros(const IOSBWaypointList &list) -> ros_t {
    ros_t r{};
    r.header.stamp = ros::Time::now();
    r.header.frame_id = "utm";
    for (const auto &wp : list.waypoints) {
        iosb_nav_msgs::Waypoint w;
        auto heading = wp.get_heading().value_or(Heading2d{0, 0});
        Waypoint::ros_t::_pose_type p;
        p.x = wp.get_position().get_x();
        p.y = wp.get_position().get_y();
        p.theta = heading.get_heading();
        w.pose = p;
        w.radius = wp.get_position().get_radius();
        w.use_heading = wp.get_heading().has_value();
        w.heading_accuracy = wp.get_heading()->get_accuracy();
        w.wait_time = wp.get_wait_time().count();
        r.waypoints.push_back(w);
    }
    return r;
}
} // namespace ects::plugins::waypoints
