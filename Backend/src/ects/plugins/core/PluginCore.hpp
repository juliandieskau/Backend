#pragma once
/* ECTS - PluginCore.hpp
 * The core plugin provides:
 * - a way to force retransmission of all or a specific topic (retransmit)
 * - a way to query the status of the robot (EctsStatusService)
 * - a way to query the name of the robot (robot_name)
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

#include "CoreMessages.hpp"
#include "ects/ECTS.hpp"
#include "ects/Plugin.hpp"
#include "ects/Timer.hpp"
#include "ros/ros.h"

#include <optional>

namespace ects::plugins::core {
class PluginCore : public Plugin {
  public:
    auto init(ECTS &) -> void override;
    auto transmit_all() -> void override;
    auto transmit(const std::string &topic_name) -> void override;
    auto name() const -> const std::string override { return "core"; }

  private:
    struct data {
        Subscriber<retransmit> retransmit_subscriber;
        Server<EctsStatusService> status_server;
    };
    std::optional<data> data;
    static constexpr auto retransmit_topic = "/ects/retransmit";
    static constexpr auto status_topic = "/ects/ects_status";
    static constexpr auto robot_name_key = "/core/robot_name";
    static constexpr auto backend_version = "1.0.0";
};

} // namespace ects::plugins::core
