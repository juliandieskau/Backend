#include "PluginCore.hpp"
#include "ects/ECTS.hpp"

namespace ects::plugins::core {
auto PluginCore::init(ects::ECTS *ects) -> void {
  ROS_INFO_STREAM("Initializing PluginCore");
  m_timer = ects->m_timerManager->createTimer(
      1.0, [this] { ROS_INFO("Timer callback"); });
  ROS_INFO("Timer created");
};
auto PluginCore::transmit_all() -> void {
  ROS_INFO_STREAM("Transmitting all");
};
auto PluginCore::transmit(std::string &topic_name) -> void {
  ROS_INFO_STREAM("Transmitting " << topic_name);
};
}; // namespace ects::plugins::core