#pragma once
#include "Configuration.hpp"
#include "RosNetworking.hpp"
#include "Timer.hpp"
namespace ects {
class ECTS {
public:
  ECTS(Configuration *config, RosInterface *rosIf, TimerManager *timerManager)
      : m_config(config), m_rosIf(rosIf), m_timerManager(timerManager) {}

  // private:
  Configuration *m_config;
  RosInterface *m_rosIf;
  TimerManager *m_timerManager;
};
} // namespace ects