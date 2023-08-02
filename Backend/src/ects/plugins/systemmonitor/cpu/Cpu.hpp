#pragma once
#include "../Usage.hpp"
#include "CpuMessages.hpp"

namespace ects::plugins::systemmonitor {
class Cpu : UsageProvider<CpuUsageMessage> {
    auto get_usage() -> CpuUsageMessage override;
};

} // namespace ects::plugins::systemmonitor
