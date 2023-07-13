#include "PluginCore.hpp"

namespace ects::plugins::core {
auto PluginCore::init() -> void { ROS_INFO_STREAM("Initializing PluginCore"); };
auto PluginCore::transmit_all() -> void {
  ROS_INFO_STREAM("Transmitting all");
};
auto PluginCore::transmit(std::string &topic_name) -> void {
  ROS_INFO_STREAM("Transmitting " << topic_name);
};
}; // namespace ects::plugins::core