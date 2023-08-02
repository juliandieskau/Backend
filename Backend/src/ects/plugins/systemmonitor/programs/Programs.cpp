#include "Programs.hpp"
#include <sys/sysinfo.h>

namespace ects::plugins::systemmonitor {

auto Programs::get_usage() -> ProcessTotalMessage {
    struct sysinfo info;
    sysinfo(&info);
    return {info.procs};
}

} // namespace ects::plugins::systemmonitor
