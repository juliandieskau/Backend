#pragma once
/* ECTS - ProgramsMessages.hpp
 * Message definition for the total number of processes.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

#include "../Usage.hpp"
#include "../aggregations/AggregationMessage.hpp"
#include "ects/ProcessTotal.h"
#include "ects/ProcessTotalHistory.h"

#include "ects/MessageInterface.hpp"
#include <string>
namespace ects::plugins::systemmonitor {

class ProcessTotalMessage : public UsageData {
  public:
    using ros_t = ects::ProcessTotal;
    using to_ros_t = ros_t;
    using history_ros_t = ects::ProcessTotalHistory;

    static auto to_ros(const ProcessTotalMessage &) -> ros_t;

    ProcessTotalMessage(uint32_t total) : UsageData(), _total(total) {}

    auto get_total() const -> uint32_t { return _total; }

    auto operator+(const ProcessTotalMessage &rhs) const
        -> ProcessTotalMessage {
        return {_total + rhs._total};
    }
    auto operator/(const int &rhs) const -> ProcessTotalMessage {
        return {_total / rhs};
    }

  private:
    uint32_t _total;
};
} // namespace ects::plugins::systemmonitor
