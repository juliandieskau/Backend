#pragma once
#include "ects/ECTS.hpp"
#include "ects/Plugin.hpp"
#include "ects/Timer.hpp"
#include "ros/ros.h"

namespace ects::plugins::core {
class PluginCore : public Plugin {
public:
  auto init(ECTS *) -> void override;
  auto transmit_all() -> void override;
  auto transmit(std::string &topic_name) -> void override;

private:
  std::shared_ptr<ects::Timer> m_timer;
  RosServer m_statusServer;
};
} // namespace ects::plugins::core