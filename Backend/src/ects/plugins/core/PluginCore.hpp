#pragma once

#include "core_messages.hpp"
#include "ects/ECTS.hpp"
#include "ects/Plugin.hpp"
#include "ects/Timer.hpp"
#include "ros/ros.h"

#include <optional>

namespace ects::plugins::core {
class PluginCore : public Plugin {
public:
  auto init(ECTS *) -> void override;

  auto transmit_all() -> void override;

  auto transmit(std::string &topic_name) -> void override;

private:
  struct data {
    subscriber<retransmit> retransmit_subscriber;
    server<ects_status_service> status_server;
  };
  std::optional<data> data;
};

} // namespace ects::plugins::core
