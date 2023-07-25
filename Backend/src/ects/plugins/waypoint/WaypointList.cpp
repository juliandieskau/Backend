#include "WaypointList.hpp"
#include "ros/ros.h"

namespace ects::plugins::waypoints {

Index::Index(size_t index) : index(index) {}
size_t Index::get() { return index; }

WaypointList::WaypointList() : waypoints(), cyclic(false) {}
void WaypointList::add_waypoint(Waypoint w, Index i) {
    if (!in_bounds(i, "add"))
        return;
    waypoints.insert(waypoints.begin() + i.get(), w);
}
void WaypointList::remove_waypoint(Index i) {
    if (!in_bounds(i, "remove"))
        return;
    waypoints.erase(waypoints.begin() + i.get());
}
void WaypointList::replace_waypoint(Index i, Waypoint w) {
    if (!in_bounds(i, "replace"))
        return;
    waypoints[i.get()] = w;
}
void WaypointList::reorder_waypoints(std::vector<Index> permutation) {
    if (permutation.size() != size()) {
        ROS_ERROR_STREAM("permutation size mismatch [permutation size: "
                         << permutation.size() << ", waypoints: " << size()
                         << "]");
        return;
    }
    std::vector<std::optional<Waypoint>> reordered(size());
    for (size_t i = 0; i < size(); ++i) {
        Index idx = permutation[i];
        if (!in_bounds(idx, "reorder"))
            return;
        if (reordered[idx.get()].has_value()) {
            ROS_ERROR_STREAM("permutation has duplicate index " << idx.get());
            return;
        }
        reordered[idx.get()] = waypoints[i];
    }
    for (size_t i = 0; i < size(); ++i)
        waypoints[i] = *reordered[i];
}
void WaypointList::reverse_waypoints() {
    std::reverse(waypoints.begin(), waypoints.end());
}
void WaypointList::set_repeat(bool repeat) { cyclic = repeat; }
double WaypointList::total_length() {
    return 0; // TODO
}
std::size_t WaypointList::size() { return waypoints.size(); }
WaypointList WaypointList::from_ros(WaypointList::ros_t ros_input) {
    WaypointList list;
    list.set_repeat(ros_input.cyclic);
    for (const auto &ros_wp : ros_input.waypoints)
        list.waypoints.push_back(Waypoint::from_ros(ros_wp));
    return list;
}
WaypointList::ros_t WaypointList::to_ros(WaypointList list) {
    WaypointList::ros_t r{};
    r.cyclic = list.cyclic;
    for (const auto &wp : list.waypoints)
        r.waypoints.push_back(Waypoint::to_ros(wp));
    return r;
}
bool WaypointList::in_bounds(Index i, std::string name) {
    if (i.get() > size()) {
        ROS_ERROR_STREAM(name << ": index out of bounds [index: " << i.get()
                              << ", size: " << size() << "]");
        return false;
    }
    return true;
}

} // namespace ects::plugins::waypoints