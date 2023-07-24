#include "core_messages.hpp"

#include <utility>

retransmit retransmit::from_ros(retransmit::from_ros_t ros_input) {
    return retransmit{ros_input.reload_all ? std::nullopt : std::optional(ros_input.topic)};
}

std::optional<std::string> retransmit::get_topic() {
    return topic_name;
}

retransmit::retransmit(std::optional<std::string> topic_name)
        : topic_name(std::move(topic_name)) {
}

ects_status_service_request ects_status_service_request::from_ros(ects_status_service_request::from_ros_t) {
    return {};
}

ects_status::to_ros_t ects_status::to_ros(ects_status status) {
    ects_status::to_ros_t r;
    r.plugins_loaded = status.loaded_plugins;
    r.robot_name = status.robot_name;
    r.version = status.version;
    return r;
}

ects_status::ects_status(std::vector<std::string> loaded_plugins, std::string robot_name, std::string version)
        : loaded_plugins(std::move(loaded_plugins)),
        robot_name(std::move(robot_name)),
        version(std::move(version)) {
}

