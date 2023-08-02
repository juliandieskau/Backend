#include "Memory.hpp"

#include <sys/sysinfo.h>

namespace ects::plugins::systemmonitor {
auto Memory::get_usage() -> MemoryUsageMessage {
    struct sysinfo info;
    sysinfo(&info);
    return {info.totalram * info.mem_unit,
            (info.totalram - info.freeram) * info.mem_unit,
            info.freeram * info.mem_unit,
            info.sharedram * info.mem_unit,
            info.bufferram * info.mem_unit,
            (info.freeram + info.bufferram) * info.mem_unit,
            info.totalswap * info.mem_unit,
            (info.totalswap - info.freeswap) * info.mem_unit,
            info.freeswap * info.mem_unit};
}
} // namespace ects::plugins::systemmonitor
