#pragma once
/* ECTS - DiskMessages.hpp
 * The class Memory provides a UsageProvider, which collects Memory and swap
 * usage, as shown by the `free` command.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */
#include "MemoryMessages.hpp"

namespace ects::plugins::systemmonitor {
class Memory : public UsageProvider<MemoryUsageMessage> {
  public:
    auto get_usage() -> MemoryUsageMessage override;
};

} // namespace ects::plugins::systemmonitor
