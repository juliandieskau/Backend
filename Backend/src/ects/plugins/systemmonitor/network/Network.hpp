#pragma once
#include "NetworkMessages.hpp"
#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

namespace ects::plugins::systemmonitor {
struct NetworkStat {
    std::chrono::time_point<std::chrono::system_clock> timestamp;
    uint64_t rx_bytes;
    uint64_t tx_bytes;
};
class Network {
  public:
    auto get_adapters() -> std::vector<std::string>;
    auto get_info(const std::string &adapter) -> NetworkInfoMessage;
    auto get_usage(const std::string &adapter) -> NetworkUsageMessage;

  private:
    static auto is_ifname_wifi(const std::string &ifname) -> bool;
    std::unordered_map<std::string, NetworkStat> last_measurements;
};

} // namespace ects::plugins::systemmonitor
