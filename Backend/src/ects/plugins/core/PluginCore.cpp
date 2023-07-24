#include "PluginCore.hpp"

namespace ects::plugins::core {
auto PluginCore::init(ects::ECTS *ects) -> void {
  ROS_INFO_STREAM("Initializing PluginCore");
  // m_timer = ects->m_timerManager->createTimer(
  //     1.0, [this] { ROS_INFO("Timer callback"); });
  // ROS_INFO("Timer created");
  m_ects = ects;
  m_nodeHandle = ros::NodeHandle();
  m_statusServer = m_nodeHandle.advertiseService(
      "/ects/ects_status", &PluginCore::statusCallback, this);
};

auto PluginCore::statusCallback(ects::ECTSStatus::Request &req,
                                ects::ECTSStatus::Response &res) -> bool {
  ROS_INFO_STREAM("Status callback");
  res.version = "0.0.1";
  res.plugins_loaded = m_ects->m_config->get_value<std::vector<std::string>>(
      "/core/load_plugins");
  res.robot_name = m_ects->m_config->get_value<std::string>("/core/robot_name");
  return true;
}

auto PluginCore::transmit_all() -> void {
  ROS_INFO_STREAM("Transmitting all");
};
auto PluginCore::transmit(std::string &topic_name) -> void {
  ROS_INFO_STREAM("Transmitting " << topic_name);
};
}; // namespace ects::plugins::core