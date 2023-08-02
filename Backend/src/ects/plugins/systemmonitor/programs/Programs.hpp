#pragma once
#include "ProgramMessages.hpp"

namespace ects::plugins::systemmonitor {
class Programs : public UsageProvider<ProcessTotalMessage> {
  public:
    auto get_usage() -> ProcessTotalMessage override;
};

} // namespace ects::plugins::systemmonitor
