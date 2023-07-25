#pragma once

#include "ects/Plugin.hpp"
#include "battery_messages.hpp"

namespace ects::plugins::battery {

class battery_monitor : public Plugin {
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
        std::optional<battery_state> state;
        subscriber<battery_state> battery_subscriber;
        publisher<battery_state> battery_state_publisher;
        publisher<charge_percentage> percentage_publisher;
        publisher<estimated_runtime> runtime_publisher;
        publisher<warning> warning_publisher;
    };
    std::optional<data> data;
    static constexpr auto battery_state_topic = "/ects/battery/usage";
    static constexpr auto percentage_topic = "/etcs/battery/percentage";
    static constexpr auto runtime_topic = "/ects/battery/estimated_time_remaining";
    static constexpr auto warning_topic = "/etcs/battery/is_critical";
};

}
