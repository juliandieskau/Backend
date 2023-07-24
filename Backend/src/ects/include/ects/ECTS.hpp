#pragma once
#include "Configuration.hpp"
#include "ros_interface.h"
#include "Timer.hpp"
namespace ects {
class ECTS {
public:
  ECTS(Configuration *config, ros_node& rosIf, TimerManager *timerManager)
      : m_config(config), m_rosIf(rosIf), m_timerManager(timerManager) {}

  // private:
  Configuration *m_config;
  ros_node& m_rosIf;
  TimerManager *m_timerManager;
};
} // namespace ects