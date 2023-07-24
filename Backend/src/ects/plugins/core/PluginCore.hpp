#pragma once
#include "ects/ECTS.hpp"
#include "ects/Plugin.hpp"
#include "ects/Timer.hpp"
#include "ros/ros.h"

#include "ects/ECTS.hpp"
#include "ects/ECTSStatus.h"
namespace ects::plugins::core {
class PluginCore : public Plugin {
public:
  auto init(ECTS *) -> void override;
  auto transmit_all() -> void override;
  auto transmit(std::string &topic_name) -> void override;
  auto statusCallback(ects::ECTSStatus::Request &req,
                      ects::ECTSStatus::Response &res) -> bool;

private:
  std::shared_ptr<ects::Timer> m_timer;
  ros::NodeHandle m_nodeHandle;
  ros::ServiceServer m_statusServer;
  ECTS *m_ects;
};
} // namespace ects::plugins::core