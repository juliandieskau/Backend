#pragma once
#include "../Usage.hpp"
#include "CpuMessages.hpp"

namespace ects::plugins::systemmonitor {
struct CpuStat {
    uint64_t total = 0;
    uint64_t idle = 0;
    uint64_t non_idle = 0;
};
class Cpu : public UsageProvider<CpuUsageMessage> {
  public:
    Cpu() : last_stats() {
        // the first call to get usage does not return valid data
        (void)get_usage();
    }
    auto get_usage() -> CpuUsageMessage override;
  private:
    std::vector<CpuStat> last_stats;
};

} // namespace ects::plugins::systemmonitor
