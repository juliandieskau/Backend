#pragma once
#include "Configuration.hpp"
#include "Timer.hpp"
#include "ros_interface.h"
namespace ects {
class ECTS {
public:
  ECTS(Configuration &config, ros_node &rosIf, TimerManager &timerManager)
      : m_config(config), m_rosIf(rosIf), m_timerManager(timerManager) {}

  Configuration &getConfig() const { return m_config; }
  ros_node &getRosIf() { return m_rosIf; }
  TimerManager &getTimerManager() { return m_timerManager; }

private:
  Configuration &m_config;
  ros_node &m_rosIf;
  TimerManager &m_timerManager;
};
} // namespace ects