#pragma once
/* ECTS - Programs.hpp
 * The class Programs provides a UsageProvider, which collects the number of
 * currently running processes.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */
#include "ProgramsMessages.hpp"

namespace ects::plugins::systemmonitor {
class Programs : public UsageProvider<ProcessTotalMessage> {
  public:
    auto get_usage() -> ProcessTotalMessage override;
};

} // namespace ects::plugins::systemmonitor
