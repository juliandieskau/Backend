#pragma once
#include "ProgramMessages.hpp"

namespace ects::plugins::systemmonitor {
class Programs : UsageProvider<ProcessTotalMessage> {
    auto get_usage() -> ProcessTotalMessage override;
};

} // namespace ects::plugins::systemmonitor
