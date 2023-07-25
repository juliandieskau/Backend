#pragma once
#include "Configuration.hpp"
#include "RosInterface.h"
#include "Timer.hpp"
namespace ects {
class ECTS {
  public:
    ECTS(const Configuration &configuration, RosNode &ros_interface,
         TimerManager &timer_manager)
        : _config(configuration), _ros_interface(ros_interface),
          _timer_manager(timer_manager) {}

    const Configuration &config() { return _config; }
    RosNode &ros_interface() { return _ros_interface; }
    TimerManager &getTimerManager() { return _timer_manager; }

  private:
    const Configuration &_config;
    RosNode &_ros_interface;
    TimerManager &_timer_manager;
};
} // namespace ects
