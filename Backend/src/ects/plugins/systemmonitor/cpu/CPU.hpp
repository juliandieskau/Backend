#pragma once
#include "CpuMessages.hpp"
#include "../Usage.hpp"

namespace ects::plugins::systemmonitor {
    class Cpu : UsageProvider<CpuUsageMessage>{
        auto get_usage() -> CpuUsageMessage override;
    };

} // namespace ects::plugins::systemmonitor
