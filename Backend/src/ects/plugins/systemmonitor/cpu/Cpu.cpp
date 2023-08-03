#include "Cpu.hpp"
#include <fstream>
#include <sstream>
#include <vector>

#include "ros/ros.h"

namespace ects::plugins::systemmonitor {
auto Cpu::get_usage() -> CpuUsageMessage {
    // get cpu usage in Linux
    // https://stackoverflow.com/questions/23367857/accurate-calculation-of-cpu-usage-given-in-percentage-in-linux
    std::ifstream file("/proc/stat");
    std::vector<float> usage_percentages;
    bool first = last_stats.empty();
    for (std::string line; std::getline(file, line);) {
        if (line.substr(0, 3) == "cpu") {
            std::istringstream ss(line);
            std::string cpu;
            ss >> cpu;
            uint64_t user, nice, system, idle, iowait, irq, softirq, steal,
                guest, guest_nice;

            ss >> user >> nice >> system >> idle >> iowait >> irq >> softirq >>
                steal >> guest >> guest_nice;

            uint64_t non_idle = user + nice + system + irq + softirq + steal;
            idle = idle + iowait;
            uint64_t total = idle + non_idle;
            if (first) {
                last_stats.push_back({total, idle, non_idle});
                continue;
            }
            auto last_stat = last_stats[usage_percentages.size()];
            uint64_t totald = total - last_stat.total;
            uint64_t idled = idle - last_stat.idle;

            float usage = (float)((double)(totald - idled) / (double)totald);

            last_stats[usage_percentages.size()] = {total, idle, non_idle};
            usage_percentages.push_back(usage);
        }
    }
    float total_usage = 0;
    if (!usage_percentages.empty()) {
        total_usage = usage_percentages[0];
        usage_percentages.erase(usage_percentages.begin());
    }

    std::ifstream load_file("/proc/loadavg");
    float load1, load5, load15;
    load_file >> load1 >> load5 >> load15;

    return {total_usage, usage_percentages, {load1, load5, load15}};
}

} // namespace ects::plugins::systemmonitor
