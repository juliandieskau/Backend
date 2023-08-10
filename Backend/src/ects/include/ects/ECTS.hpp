#pragma once

/* ECTS - ECTS.hpp
 * Extensible Control and Telemetry System
 * This file contains the main class, ECTS, which encompasses the entire API
 * exposed to ECTS plugins.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik Oelbracht, Liam
 * Derk Rembold
 */

#include "ects/Configuration.hpp"
#include "ects/RosInterface.hpp"
#include "ects/Timer.hpp"
#include <ects/Plugin.hpp>
#include <string>
#include <vector>

namespace ects {
class ECTS {
  public:
    ECTS(const Configuration &configuration, RosNode &ros_interface,
         TimerManager &timer_manager)
        : _config(configuration), _ros_interface(ros_interface),
          _timer_manager(timer_manager), plugins() {}

    auto config() const -> const Configuration & { return _config; }
    auto ros_interface() -> RosNode & { return _ros_interface; }
    auto timer_manager() -> TimerManager & { return _timer_manager; }
    auto get_plugins() -> const std::vector<std::unique_ptr<Plugin>> & {
        return plugins;
    }
    auto add_plugin(std::unique_ptr<Plugin> plugin) -> void {
        plugins.push_back(std::move(plugin));
    }

  private:
    std::vector<std::unique_ptr<Plugin>> plugins;
    const Configuration &_config;
    RosNode &_ros_interface;
    TimerManager &_timer_manager;
};
} // namespace ects
