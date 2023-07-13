#pragma once
#include "Configuration.hpp"
#include "RosNode.hpp"
#include "TimerManager.hpp"
namespace ects {
class ECTS {
  ECTS(Configuration config, RosNode rosNode, TimerManager timerManager);
};
} // namespace ects