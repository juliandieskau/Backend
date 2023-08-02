#include "Cpu.hpp"
#include <fstream>
#include <sstream>
#include <vector>

namespace ects::plugins::systemmonitor {
auto Cpu::get_usage() -> CpuUsageMessage {
    // get cpu usage in Linux
    std::ifstream file("/proc/stat");
    std::vector<float> usage_percentages;
    for (std::string line; std::getline(file, line);) {
        if (line.substr(0, 3) == "cpu") {
            std::istringstream ss(line);
            std::string cpu;
            ss >> cpu;
            size_t user, nice, system, idle;
            size_t total;
            ss >> user >> nice >> system >> idle;
            total = user + nice + system + idle;
            usage_percentages.push_back(1.0 - (idle * 1.0 / total));
        }
    }
    float total_usage = usage_percentages[0];
    usage_percentages.erase(usage_percentages.begin());

    std::ifstream load_file("/proc/loadavg");
    float load1, load5, load15;
    load_file >> load1 >> load5 >> load15;

    return {total_usage, usage_percentages, {load1, load5, load15}};
}

} // namespace ects::plugins::systemmonitor
