#pragma once
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
    auto get_plugins() -> std::vector<Plugin *> {
        return std::vector<Plugin *>(plugins);
    }
    auto add_plugin(Plugin *plugin) -> void { plugins.push_back(plugin); }

  private:
    std::vector<Plugin *> plugins;
    const Configuration &_config;
    RosNode &_ros_interface;
    TimerManager &_timer_manager;
};
} // namespace ects
