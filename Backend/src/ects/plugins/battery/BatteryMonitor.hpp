#pragma once

#include "BatteryMessages.hpp"
#include "ects/ECTS.hpp"
#include "ects/Plugin.hpp"

namespace ects::plugins::battery {

class BatteryMonitor : public Plugin {
  public:
    auto init(ECTS *) -> void override;
    auto transmit_all() -> void override;
    auto transmit(std::string &topic_name) -> void override;

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
    static constexpr auto percentage_topic = "/etcs/battery/percentage";
    static constexpr auto runtime_topic =
        "/ects/battery/estimated_time_remaining";
    static constexpr auto warning_topic = "/etcs/battery/is_critical";
};

} // namespace ects::plugins::battery
