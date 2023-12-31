#pragma once
/* ECTS - BatteryMonitor.hpp
 * The battery plugin provides a way to monitor the battery of the robot,
 * and issue a warning if the battery is critcally low.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

#include "BatteryMessages.hpp"
#include "ects/ECTS.hpp"
#include "ects/Plugin.hpp"

namespace ects::plugins::battery {

class BatteryMonitor : public Plugin {
  public:
    auto init(ECTS &) -> void override;
    auto transmit_all() -> void override;
    auto transmit(const std::string &topic_name) -> void override;
    auto name() const -> const std::string override { return "battery"; }

  private:
    auto publish_battery_state() -> void;
    auto publish_percentage() -> void;
    auto publish_runtime() -> void;
    auto publish_warning() -> void;

    struct data {
        std::optional<BatteryState> state;
        Subscriber<BatteryState> battery_subscriber;
        Publisher<BatteryState> battery_state_publisher;
        Publisher<ChargePercentage> percentage_publisher;
        Publisher<EstimatedRuntime> runtime_publisher;
        Publisher<Warning> warning_publisher;
    };
    std::optional<data> data;
    static constexpr auto battery_state_topic = "/ects/battery/usage";
    static constexpr auto percentage_topic = "/ects/battery/percentage";
    static constexpr auto runtime_topic =
        "/ects/battery/estimated_time_remaining";
    static constexpr auto warning_topic = "/ects/battery/is_critical";
    static constexpr auto battery_topic_key = "/battery/battery_topic";
};

} // namespace ects::plugins::battery
