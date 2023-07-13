#pragma once
#include "ects/Plugin.hpp"
#include "ros/ros.h"

namespace ects::plugins::core {
class PluginCore : public Plugin {
  auto init() -> void override;
  auto transmit_all() -> void override;
  auto transmit(std::string &topic_name) -> void override;
};
} // namespace ects::plugins::core