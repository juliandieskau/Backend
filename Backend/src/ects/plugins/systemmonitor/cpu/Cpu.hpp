#pragma once
#include "../Usage.hpp"
#include "CpuMessages.hpp"

namespace ects::plugins::systemmonitor {
class Cpu : public UsageProvider<CpuUsageMessage> {
  public:
    auto get_usage() -> CpuUsageMessage override;
};

} // namespace ects::plugins::systemmonitor
