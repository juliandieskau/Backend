#pragma once
#include "NetworkMessages.hpp"
#include <string>
#include <vector>

namespace ects::plugins::systemmonitor {

class Network {
    auto get_info() -> NetworkInfoMessage;
    auto get_adapters() -> std::vector<std::string>;
    auto get_usage(const std::string &adapter) -> NetworkUsageMessage;
};

} // namespace ects::plugins::systemmonitor
