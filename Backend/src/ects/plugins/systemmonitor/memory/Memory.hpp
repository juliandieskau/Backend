#pragma once
#include "MemoryMessages.hpp"

namespace ects::plugins::systemmonitor {
class Memory : public UsageProvider<MemoryUsageMessage> {
  public:
    auto get_usage() -> MemoryUsageMessage override;
};

} // namespace ects::plugins::systemmonitor
