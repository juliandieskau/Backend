#include "CoreMessages.hpp"
#include <utility>

namespace ects::plugins::core {

retransmit retransmit::from_ros(const retransmit::from_ros_t &ros_input) {
    return retransmit{ros_input.reload_all ? std::nullopt
                                           : std::optional(ros_input.topic)};
}

std::optional<std::string> retransmit::get_topic() { return topic_name; }

retransmit::retransmit(std::optional<std::string> topic_name)
    : topic_name(std::move(topic_name)) {}

EctsStatus::to_ros_t EctsStatus::to_ros(const EctsStatus &status) {
    EctsStatus::to_ros_t r;
    r.plugins_loaded = status.loaded_plugins;
    r.robot_name = status.robot_name;
    r.version = status.version;
    return r;
}

EctsStatus::EctsStatus(std::vector<std::string> loaded_plugins,
                       std::string robot_name, std::string version)
    : loaded_plugins(std::move(loaded_plugins)),
      robot_name(std::move(robot_name)), version(std::move(version)) {}

} // namespace ects::plugins::core
