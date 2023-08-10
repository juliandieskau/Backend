#include "WaypointMessages.hpp"

#include <utility>

namespace ects::plugins::waypoints {

auto AddWaypointMessage::get_waypoint() -> Waypoint { return waypoint; }
auto AddWaypointMessage::get_index() -> Index { return index; }
auto AddWaypointMessage::from_ros(const AddWaypointMessage::ros_t &ros_input)
    -> AddWaypointMessage {
    return {{ros_input.index}, Waypoint::from_ros(ros_input.waypoint)};
}
AddWaypointMessage::AddWaypointMessage(Index index, Waypoint waypoint)
    : index(index), waypoint(std::move(waypoint)) {}

auto RemoveWaypointMessage::get_index() -> Index { return index; }
RemoveWaypointMessage::RemoveWaypointMessage(Index index) : index(index) {}
auto RemoveWaypointMessage::from_ros(
    const RemoveWaypointMessage::ros_t &ros_input) -> RemoveWaypointMessage {
    return {Index{ros_input.index}};
}

auto ReplaceWaypointMessage::get_index() -> Index { return index; }
auto ReplaceWaypointMessage::get_waypoint() -> Waypoint { return waypoint; }
auto ReplaceWaypointMessage::from_ros(
    const ReplaceWaypointMessage::ros_t &ros_input) -> ReplaceWaypointMessage {
    return {Index{ros_input.index_to_replace},
            Waypoint::from_ros(ros_input.replacement_waypoint)};
}
ReplaceWaypointMessage::ReplaceWaypointMessage(Index index, Waypoint waypoint)
    : index(index), waypoint(std::move(waypoint)) {}

auto ReorderWaypointsMessage::get_permutation() -> std::vector<Index> {
    return permutation;
}
auto ReorderWaypointsMessage::from_ros(
    const ReorderWaypointsMessage::ros_t &ros_input)
    -> ReorderWaypointsMessage {
    ReorderWaypointsMessage reorder_message{};
    for (auto index : ros_input.new_indices)
        reorder_message.permutation.emplace_back(index);
    return reorder_message;
}

auto RepeatWaypointsMessage::get_repeat() const -> bool { return repeat; }
auto RepeatWaypointsMessage::from_ros(
    const RepeatWaypointsMessage::ros_t &ros_input) -> RepeatWaypointsMessage {
    return {static_cast<bool>(ros_input.data)};
}
RepeatWaypointsMessage::RepeatWaypointsMessage(bool repeat) : repeat(repeat) {}

NumberOfWaypointsMessage::NumberOfWaypointsMessage(size_t waypoints)
    : count(waypoints) {}
auto NumberOfWaypointsMessage::to_ros(
    const NumberOfWaypointsMessage &internal_input)
    -> NumberOfWaypointsMessage::ros_t {
    NumberOfWaypointsMessage::ros_t r{};
    r.data = internal_input.count;
    return r;
}

} // namespace ects::plugins::waypoints
