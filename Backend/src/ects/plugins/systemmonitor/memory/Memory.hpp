#pragma once
#include "MemoryMessages.hpp"

namespace ects::plugins::systemmonitor {
class Memory : UsageProvider<MemoryUsageMessage> {
    auto get_usage() -> MemoryUsageMessage override;
};

} // namespace ects::plugins::systemmonitor
