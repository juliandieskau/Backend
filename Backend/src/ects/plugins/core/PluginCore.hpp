#pragma once
#include "ects/ECTS.hpp"
#include "ects/Plugin.hpp"
#include "ros/ros.h"

namespace ects::plugins::core {
class PluginCore : public Plugin {
  auto init(ECTS *) -> void override;
  auto transmit_all() -> void override;
  auto transmit(std::string &topic_name) -> void override;
};
} // namespace ects::plugins::core